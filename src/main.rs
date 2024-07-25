#![no_std]
#![no_main]

mod audio;
mod fs;
mod fx;
mod pwm;
mod utils;

use audio::steps::Steps;
use audio::steps::Event;
use audio::WavWriter;
use pwm::InterruptPwm;
use utils::AsyncMutex;

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;
use alloc::vec::Vec;
use alloc::boxed::Box;

use core::cell::RefCell;

use defmt::*;
use micromath::F32Ext;

use cortex_m::peripheral::NVIC;

use embassy_executor::{SendSpawner, Spawner};
use embassy_sync::blocking_mutex::CriticalSectionMutex as SyncMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::Timer;

use embassy_stm32::{gpio, interrupt, sdmmc};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::{ADC1, TIM4};
use embassy_stm32::time::Hertz;

pub const PAD_COUNT: u8 = 8;
const INPUT_CHANNEL_LEN: usize = 16;
const INPUT_BUFFER_LEN: usize = 512;

type RingBuf<T, const CAP: usize> = arraydeque::ArrayDeque<T, CAP, arraydeque::Wrapping>;

static ADC: SyncMutex<RefCell<Option<embassy_stm32::adc::Adc<ADC1>>>> = SyncMutex::new(RefCell::new(None));
static PWM: SyncMutex<RefCell<Option<InterruptPwm<TIM4>>>> = SyncMutex::new(RefCell::new(None));
static SPAWNER: SyncMutex<RefCell<Option<SendSpawner>>> = SyncMutex::new(RefCell::new(None));
static RECORD_SW: SyncMutex<RefCell<Option<gpio::Input>>> = SyncMutex::new(RefCell::new(None));

type AudioInPin = embassy_stm32::peripherals::PA1;
static AUDIO_IN_PIN: SyncMutex<RefCell<Option<AudioInPin>>> = SyncMutex::new(RefCell::new(None));

static VOL_MGR: AsyncMutex<Option<fs::VolMgr>> = AsyncMutex::new(None);
static ROOT_DIR: AsyncMutex<Option<embedded_sdmmc::RawDirectory>> = AsyncMutex::new(None);
static RECORDER: AsyncMutex<Option<WavWriter>> = AsyncMutex::new(None);
static STEPS: AsyncMutex<Option<Steps>> = AsyncMutex::new(None);
static BITCRUSH: AsyncMutex<Option<fx::Bitcrush>> = AsyncMutex::new(None);

static MTX: AsyncMutex<RefCell<Vec<u8>>> = AsyncMutex::new(RefCell::new(Vec::new()));

embassy_stm32::bind_interrupts!(struct Irqs {
    SDIO => sdmmc::InterruptHandler<embassy_stm32::peripherals::SDIO>;
});

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("init heap...");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 64000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL192,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV4),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
    }

    let p = embassy_stm32::init(config);

    info!("init sdmmc...");
    let mut sdmmc = embassy_stm32::sdmmc::Sdmmc::new_4bit(
        p.SDIO,
        Irqs,
        p.DMA2_CH3,
        p.PB15,
        p.PA6,
        p.PB4,
        p.PA8,
        p.PA9,
        p.PB5,
        Default::default(),
    );

    let mut err = None;
    while let Err(e) = sdmmc.init_card(Hertz::mhz(16)).await {
        if err != Some(e) {
            info!("waiting for card error, retrying: {:?}", e);
            err = Some(e);
        }
    }

    let card = fs::SdioCard::new(sdmmc);

    let mut vol_mgr = embedded_sdmmc::VolumeManager::new_with_limits(
        card,
        fs::DummyTimesource::default(),
        0,
    );
    let volume = crate::expect!(
        vol_mgr.open_raw_volume(embedded_sdmmc::VolumeIdx(0)).await,
        "failed to open volume 0"
    );
    let root_dir = crate::expect!(
        vol_mgr.open_root_dir(volume),
        "failed to open root directory"
    );
    ROOT_DIR.lock().await.replace(root_dir);
    VOL_MGR.lock().await.replace(vol_mgr);

    let led = gpio::Output::new(
        p.PC13,
        gpio::Level::High,
        gpio::Speed::Low,
    );
    let steps = crate::expect!(
        Steps::new(lock_async_mut!(VOL_MGR), lock_async_ref!(ROOT_DIR), led).await,
        "failed to init steps"
    );
    STEPS.lock().await.replace(steps);

    let bitcrush = fx::Bitcrush::new();
    BITCRUSH.lock().await.replace(bitcrush);

    let send_spawner = SendSpawner::for_current_executor().await;
    SPAWNER.lock(|s| s.replace(Some(send_spawner)));

    info!("init clock sync...");
    let clk_sw = ExtiInput::new(p.PB1, p.EXTI1, gpio::Pull::Up);
    let _ = spawner.spawn(handle_clock_in(clk_sw));

    info!("init mode...");
    let mode_sw = ExtiInput::new(p.PA0, p.EXTI0, gpio::Pull::Up);
    let _ = spawner.spawn(handle_mode(mode_sw));

    info!("init matrix...");
    macro_rules! new_output {
        ( $x:expr ) => {
            gpio::Output::new(
                $x,
                gpio::Level::High,
                gpio::Speed::Low,
            )
        };
    }
    macro_rules! new_input {
        ( $x:expr ) => {
            gpio::Input::new(
                $x,
                gpio::Pull::Up,
            )
        };
    }

    let mx_input0 = new_input!(p.PB3);
    let mx_input1 = new_input!(p.PB6);
    let mx_input2 = new_input!(p.PB7);
    let mx_input3 = new_input!(p.PB9);

    let mx_output0 = new_output!(p.PA10);
    let mx_output1 = new_output!(p.PA11);
    let mx_output2 = new_output!(p.PA12);
    let mx_output3 = new_output!(p.PA15);

    let _ = spawner.spawn(handle_matrix(
        [mx_input0, mx_input1, mx_input2, mx_input3],
        [mx_output0, mx_output1, mx_output2, mx_output3],
    ));

    info!("init adc...");
    let mut adc = embassy_stm32::adc::Adc::new(p.ADC1);
    adc.set_resolution(embassy_stm32::adc::Resolution::BITS8);
    adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES3);
    ADC.lock(|adc_this| adc_this.replace(Some(adc)));

    let cut_sw = new_input!(p.PB12);
    let seq_sw = new_input!(p.PB10);

    let _ = spawner.spawn(handle_joystick(
        p.PA2,
        p.PA3,
        new_input!(p.PB13),
    ));

    let _ = spawner.spawn(handle_pads(
        p.PA4,
        cut_sw,
        seq_sw,
    ));

    info!("init pwm...");
    let record_sw = gpio::Input::new(p.PB0, gpio::Pull::Up);
    RECORD_SW.lock(|p| p.replace(Some(record_sw)));
    let audio_pin = p.PA1;
    AUDIO_IN_PIN.lock(|p| p.replace(Some(audio_pin)));

    let mut pwm = InterruptPwm::new(
        p.TIM4,
        None,
        None,
        Some(embassy_stm32::timer::simple_pwm::PwmPin::new_ch3(p.PB8, gpio::OutputType::PushPull)),
        None,
        Hertz::khz(32)
    );
    let _ = pwm.enable(embassy_stm32::timer::Channel::Ch3);

    PWM.lock(|p| p.replace(Some(pwm)));
    unsafe { NVIC::unmask(embassy_stm32::interrupt::TIM4) };

    info!("idle start!");
    loop {
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn handle_clock_in(mut input: ExtiInput<'static>) {
    let mut past = None;
    let mut buffer: RingBuf<u64, 24> = RingBuf::new();
    loop {
        while embassy_time::with_timeout(
            embassy_time::Duration::from_millis(84),
            input.wait_for_rising_edge()
        ).await.is_ok() {
            let now = embassy_time::Instant::now();
            if let Some(past) = past {
                buffer.push_back(now.saturating_duration_since(past).as_micros());

                // mean of all buffered intervals scaled to higher frequency
                let micros = {
                    let max = buffer.iter().copied().max().unwrap_or(0);
                    let maxes = buffer
                        .iter()
                        .map(|&v| v as f32)
                        .filter(|&v| (max as f32 / v).round() == 1.0)
                        .collect::<Vec<_>>();
                    let mean_max = maxes.iter().sum::<f32>() / maxes.len() as f32;
                    buffer
                        .iter()
                        .map(|&v| {
                            v * (mean_max as f32 / v as f32).round() as u64
                        })
                        .sum::<u64>() as f32 / buffer.len() as f32
                };

                let tempo = 1000000.0 * 60.0 / 24.0 / micros;
                match lock_async_mut!(STEPS).set_clock_in_tempo(Some(tempo)).await {
                    Err(audio::Error::NoFileLoaded) => continue,
                    Err(e) => defmt::panic!("failed to set clock in: {}", Debug2Format(&e)),
                    _ => (),
                };
            }
            past = Some(now);
        }
        let _ = lock_async_mut!(STEPS).set_clock_in_tempo(None).await;
        buffer = RingBuf::new();
        past = None;
    }
}

#[embassy_executor::task]
async fn handle_mode(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_falling_edge().await;
        let _ = lock_async_mut!(STEPS).set_running(lock_async_mut!(VOL_MGR), false);
        input.wait_for_falling_edge().await;
        let _ = lock_async_mut!(STEPS).set_running(lock_async_mut!(VOL_MGR), true);
    }
}

/// 4x4 button matrix where mosi pins -> columns, miso pins -> rows
#[embassy_executor::task]
async fn handle_matrix(
    mx_inputs: [gpio::Input<'static>; 4],
    mut mx_outputs: [gpio::Output<'static>; 4],
) {
    let mut prev_downs = Vec::new();
    loop {
        let mut downs = prev_downs.clone();

        // process matrix
        for (o_idx, mx_output) in mx_outputs.iter_mut().enumerate() {
            mx_output.set_low();
            for (i_idx, mx_input) in mx_inputs.iter().enumerate() {
                let index = i_idx as u8 * 4 + o_idx as u8;
                if mx_input.is_high() && downs.contains(&index) {
                    downs.retain(|&v| v != index);
                } else if mx_input.is_low() && !downs.contains(&index) && index < 16 {
                    downs.push(index);
                }
            }
            mx_output.set_high();

            Timer::after_millis(1).await;
        }

        MTX.lock().await.borrow_mut().clone_from(&downs);
        prev_downs = downs;
    }
}

#[embassy_executor::task]
async fn handle_pads(
    mut select_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    cut_sw: gpio::Input<'static>,
    seq_sw: gpio::Input<'static>,
) {
    let mut buf: RingBuf<u16, 16> = RingBuf::new();
    let mut prev_downs = Vec::new();
    let mut file_loaded = false;
    let mut cut_index = None;
    let mut seq = Vec::new();
    loop {
        let downs = MTX.lock().await.borrow().clone();
        if cut_sw.is_low() {
            buf.push_back(ADC.lock(|adc| adc.borrow_mut().as_mut().unwrap().read(&mut select_pin)));

            let wma = {
                let length = buf.len();
                buf
                    .iter()
                    .enumerate()
                    .fold(0.0, |acc, (i, &v)| {
                        acc + v as f32 * (i + 1) as f32
                    }) / (length * (length + 1) / 2) as f32
            };
            let file_count = lock_async_ref!(STEPS).file_count();
            let file_index = ((wma / 255.0).clamp(0.0, 1.0) * file_count as f32) as usize;
            file_loaded = true;

            if prev_downs != downs {
                // assign steps of file to pad
                if let Some(cut_idx) = cut_index {
                    if let Some(&step_count) = downs.last() {
                        let step_count = step_count + 1;

                        match lock_async_mut!(STEPS).assign_pad(lock_async_mut!(VOL_MGR), Some(file_index), Some(cut_idx), Some(step_count)).await {
                            Err(audio::Error::StepOutOfBounds) => (),
                            Err(e) => defmt::panic!("failed to assign pad: {}", Debug2Format(&e)),
                            Ok(true) => (), // file exhausted
                            _ => (),
                        }

                        cut_index = None;
                    }
                } else {
                    cut_index = downs.last().copied();
                    info!("target cut {}...", cut_index);
                }
            }
        } else if file_loaded {
            cut_index = None;
            file_loaded = false;
            let _ = lock_async_mut!(STEPS).assign_pad(lock_async_mut!(VOL_MGR), None, None, None).await;
        } else if seq_sw.is_low() {
            let downs = MTX.lock().await.borrow().clone();
            if prev_downs != downs {
                // build sequence
                if let Some(pad_idx) = downs.last().copied() {
                    seq.push(pad_idx);
                    info!("add cut {} to sequence...", pad_idx);
                }
            }
        } else if !seq.is_empty() {
            // load sequence
            info!("init load!");
            let _ = lock_async_mut!(STEPS).buffer(Event::Load { cuts: seq });
            seq = Vec::new();
        } else {
            // step sequencing
            handle_step_sequence(prev_downs, downs.clone()).await;
        }

        prev_downs = downs;
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn handle_joystick(
    mut x_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    mut y_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    joy_sw: gpio::Input<'static>,
) {
    let mut buf: RingBuf<(u16, u16), 16> = RingBuf::new();
    // process state
    loop {
        // handle joystick
        if joy_sw.is_low() {
            let x = ADC.lock(|adc| adc.borrow_mut().as_mut().unwrap().read(&mut x_pin));
            let y = ADC.lock(|adc| adc.borrow_mut().as_mut().unwrap().read(&mut y_pin));
            buf.push_back((x, y));

            let (x, y) = {
                let length = buf.len();
                let (x, y) = buf
                    .iter()
                    .enumerate()
                    .fold((0.0, 0.0), |acc, (i, v)| (
                        acc.0 + v.0 as f32 * (i + 1) as f32,
                        acc.1 + v.1 as f32 * (i + 1) as f32
                    ));
                (
                    x / (length * (length + 1) / 2) as f32,
                    y / (length * (length + 1) / 2) as f32
                )
            };
            let bit_depth = (x + 255.0) * (x - 255.0) / 255.0.powi(2) + 1.0;
            let rate = (y + 255.0) * (y - 255.0) / 255.0.powi(2) + 1.0;
            lock_async_mut!(BITCRUSH).set_bit_depth(bit_depth);
            lock_async_mut!(BITCRUSH).set_rate(rate);
        }

        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn read(sender: Sender<'static, RawMutex, Box<[u8]>, 1>) {
    let mut grain = match lock_async_mut!(STEPS).read_grain(lock_async_mut!(VOL_MGR)).await {
        Err(audio::Error::NoFileLoaded) => Box::new([u8::MAX / 2; 128]),
        Err(e) => defmt::panic!("read err: {}", Debug2Format(&e)),
        Ok(v) => v,
    };
    lock_async_ref!(BITCRUSH).crush(&mut grain);
    sender.send(grain).await;
}

#[embassy_executor::task]
async fn record_sample(receiver: Receiver<'static, RawMutex, Vec<u8>, INPUT_CHANNEL_LEN>) {
    if RECORDER.lock().await.is_none() {
        info!("starting recording...");
        let recorder = crate::expect!(
            WavWriter::new(crate::lock_async_mut!(VOL_MGR), crate::lock_async_ref!(ROOT_DIR), "input").await,
            "failed to init recorder"
        );
        RECORDER.lock().await.replace(recorder);
    }
    if let Some(recorder) = RECORDER.lock().await.as_mut() {
        while let Ok(samples) = receiver.try_receive() {
            crate::expect!(
                recorder.write(crate::lock_async_mut!(VOL_MGR), &samples).await,
                "failed to write samples"
            );
        }
    }
}

#[embassy_executor::task]
async fn bake_recording(
    mut recorder: WavWriter,
    receiver: Receiver<'static, RawMutex, Vec<u8>, INPUT_CHANNEL_LEN>
) {
    info!("baking recording...");
    while let Ok(samples) = receiver.try_receive() {
        crate::expect!(
            recorder.write(crate::lock_async_mut!(VOL_MGR), &samples).await,
            "failed to write samples"
        );
    }

    let wav = crate::expect!(
        recorder.release(crate::lock_async_mut!(VOL_MGR), crate::lock_async_ref!(ROOT_DIR)).await,
        "failed to bake recording"
    );

    defmt::info!("baked recording!");
    lock_async_mut!(STEPS).push_wav(wav).await;
}

#[allow(clippy::too_many_arguments)]
#[interrupt]
fn TIM4() {
    static OUTPUT_CHANNEL: Channel<RawMutex, Box<[u8]>, 1> = Channel::new();
    static mut OUTPUT_BUFFER: Option<Box<[u8]>> = None;
    static mut INDEX: usize = 0;
    static mut PWM_THIS: Option<InterruptPwm<TIM4>> = None;

    static INPUT_CHANNEL: Channel<RawMutex, Vec<u8>, INPUT_CHANNEL_LEN> = Channel::new();
    static mut INPUT_BUFFER: Option<Vec<u8>> = None;
    static mut INSTANT: Option<embassy_time::Instant> = None;
    static mut AUDIO_IN_PIN_THIS: Option<AudioInPin> = None;
    static mut RECORD_SW_THIS: Option<gpio::Input> = None;

    let audio_in_pin = AUDIO_IN_PIN_THIS.get_or_insert_with(|| {
        AUDIO_IN_PIN.lock(|p| p.take().unwrap())
    });
    let record_sw = RECORD_SW_THIS.get_or_insert_with(|| {
        RECORD_SW.lock(|p| p.take().unwrap())
    });

    if record_sw.is_low() {
        let instant = INSTANT.get_or_insert_with(embassy_time::Instant::now);

        if embassy_time::Instant::now().duration_since(*instant).as_millis() > 17 {
            let input_buffer = INPUT_BUFFER.get_or_insert_with(Vec::new);

            let input = ADC.lock(|adc| adc.borrow_mut().as_mut().unwrap().read(audio_in_pin));
            let sample = input as u8;

            input_buffer.push(sample);
            if input_buffer.len() >= INPUT_BUFFER_LEN {
                let _ = INPUT_CHANNEL.try_send(input_buffer.clone());
                let _ = SPAWNER.lock(|s| {
                    s
                        .borrow_mut()
                        .unwrap()
                        .spawn(record_sample(INPUT_CHANNEL.receiver()))
                });
                *INPUT_BUFFER = None;
            }
        }
    } else if let Ok(Some(recorder)) = RECORDER.try_lock().map(|mut opt| opt.take()) {
        *INSTANT = None;

        let _ = SPAWNER.lock(|s| {
            s
                .borrow_mut()
                .unwrap()
                .spawn(bake_recording(recorder, INPUT_CHANNEL.receiver()))
        });
    }

    let buffer = OUTPUT_BUFFER.get_or_insert_with(|| Box::new([]));
    let pwm = PWM_THIS.get_or_insert_with(|| {
        PWM.lock(|pwm| pwm.take().unwrap())
    });

    if *INDEX >= buffer.len() {
        *INDEX = 0;
        if OUTPUT_CHANNEL.is_full() {
            *buffer = OUTPUT_CHANNEL.try_receive().unwrap();
        }
        let _ = SPAWNER.lock(|s| {
            s
                .borrow_mut()
                .unwrap()
                .spawn(read(OUTPUT_CHANNEL.sender()))
        });
    }

    let sample = buffer.get(*INDEX).copied().unwrap_or(u8::MAX / 2);
    *INDEX += 1;

    let duty = (sample as f32 / u8::MAX as f32 * pwm.get_max_duty() as f32) as u32;

    if let Err(e) = pwm.set_duty(embassy_stm32::timer::Channel::Ch3, duty) {
        defmt::panic!("failed to set duty: {}", Debug2Format(&e));
    }

    pwm.clear_update_interrupt();
}

async fn handle_step_sequence(
    prev_downs: Vec<u8>,
    mut downs: Vec<u8>,
) {
    let step_count = lock_async_ref!(STEPS).step_count().unwrap_or(0);
    downs.retain(|&v| v < PAD_COUNT);
    // process state
    if prev_downs != downs {
        match downs.first() {
            Some(&step) if step < step_count => {
                if downs.len() > 1 {
                    // init loop start
                    let numerator = downs
                        .iter()
                        .skip(1)
                        .map(|v| v.checked_sub(step + 1).unwrap_or(v + PAD_COUNT - 1 - step))
                        .fold(0u32, |acc, v| acc | 1 << v);

                    info!("numerator: {}, downs: {}", numerator, Debug2Format(&downs));
                    let length = utils::Fraction::new(numerator, 16);

                    let _ = lock_async_mut!(STEPS).buffer(Event::Loop { step: step as u32, length });
                } else if prev_downs.len() > 1 {
                    // init loop stop
                    let _ = lock_async_mut!(STEPS).buffer(Event::Sync);
                } else {
                    // init jump
                    let _ = lock_async_mut!(STEPS).buffer(Event::Hold { step: step as u32 });
                }
            },
            _ => {
                // init sync
                let _ = lock_async_mut!(STEPS).buffer(Event::Sync);
            },
        }
    }
}
