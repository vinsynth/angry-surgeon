#![no_std]
#![no_main]

mod audio;
mod fs;
mod pwm;
mod rhythm;
mod utils;

use audio::GRAIN_LEN;
use pwm::InterruptPwm;
use utils::AsyncMutex;

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;
use alloc::vec::Vec;

use core::cell::RefCell;

use defmt::*;

use cortex_m::peripheral::NVIC;

use embassy_executor::{SendSpawner, Spawner};
use embassy_sync::blocking_mutex::CriticalSectionMutex as SyncMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::Timer;

use embassy_stm32::{gpio, interrupt, sdmmc};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::{ADC1, TIM4};
use embassy_stm32::time::Hertz;

pub const PAD_COUNT: u8 = 8;

type RingBuf<T, const CAP: usize> = arraydeque::ArrayDeque<T, CAP, arraydeque::Wrapping>;

static PWM: SyncMutex<RefCell<Option<InterruptPwm<TIM4>>>> = SyncMutex::new(RefCell::new(None));
static SPAWNER: SyncMutex<RefCell<Option<SendSpawner>>> = SyncMutex::new(RefCell::new(None));

static ADC: AsyncMutex<Option<embassy_stm32::adc::Adc<ADC1>>> = AsyncMutex::new(None);
static MTX: AsyncMutex<RefCell<Vec<u8>>> = AsyncMutex::new(RefCell::new(Vec::new()));
static STEPS: AsyncMutex<Option<audio::Steps>> = AsyncMutex::new(None);

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

    let mut vol_mgr = embedded_sdmmc::VolumeManager::new(card, fs::DummyTimesource::default());

    let led = gpio::Output::new(
        p.PC13,
        gpio::Level::High,
        gpio::Speed::Low,
    );
    let steps = crate::expect!(
        audio::Steps::new(led, vol_mgr).await,
        "failed to init steps"
    );
    STEPS.lock().await.replace(steps);

    let send_spawner = SendSpawner::for_current_executor().await;
    SPAWNER.lock(|s| s.replace(Some(send_spawner)));

    info!("init pwm...");
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

    info!("init tapper...");
    let tap_sw = ExtiInput::new(p.PB14, p.EXTI14, gpio::Pull::Up);
    let _ = spawner.spawn(handle_tapper(tap_sw));

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
    let adc = embassy_stm32::adc::Adc::new(p.ADC1);
    ADC.lock().await.replace(adc);

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

    info!("idle start!");
    loop {
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn handle_mode(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_falling_edge().await;
        let _ = lock_async_mut!(STEPS).set_running(false);
        input.wait_for_falling_edge().await;
        let _ = lock_async_mut!(STEPS).set_running(true);
    }
}

#[embassy_executor::task]
async fn handle_tapper(mut input: ExtiInput<'static>) {
    'outer: loop {
        input.wait_for_falling_edge().await;
        let mut prev = match lock_async_mut!(STEPS).tap(None, false).await {
            Ok(Some(v)) => v,
            Err(audio::Error::NoFileLoaded) => continue,
            Err(e) => defmt::panic!("failed to tap: {}", Debug2Format(&e)),
            _ => defmt::unreachable!(),
        };

        while embassy_time::with_timeout(
            embassy_time::Duration::from_secs(2),
            input.wait_for_low()
        ).await.is_ok() {
            prev = match lock_async_mut!(STEPS).tap(Some(prev), false).await {
                Ok(Some(v)) => v,
                Err(audio::Error::NoFileLoaded) => continue 'outer,
                Err(e) => defmt::panic!("failed to tap: {}", Debug2Format(&e)),
                _ => defmt::unreachable!(),
            };
            Timer::after_millis(1).await;
        }

        info!("canceled tap!");
        let _ = lock_async_mut!(STEPS).tap(None, true).await;
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
            buf.push_back(lock_async_mut!(ADC).read(&mut select_pin));

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
            let file_index = ((wma / 4095.0).clamp(0.0, 1.0) * file_count as f32) as usize;
            file_loaded = true;

            if prev_downs != downs {
                // assign steps of file to pad
                if let Some(cut_idx) = cut_index {
                    if let Some(&step_count) = downs.last() {
                        let step_count = step_count + 1;

                        match lock_async_mut!(STEPS).assign_pad(Some(file_index), Some(cut_idx), Some(step_count)).await {
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
            let _ = lock_async_mut!(STEPS).assign_pad(None, None, None).await;
            Timer::after_millis(1).await;
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
            let _ = lock_async_mut!(STEPS).buffer(audio::Event::Load { cuts: seq });
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
            let x = lock_async_mut!(ADC).read(&mut x_pin);
            let y = lock_async_mut!(ADC).read(&mut y_pin);
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
            let speed_mod = if x > 2047.0 {
                (x / 2047.0 - 1.0) * 3.0 + 1.0
            } else {
                x / 2047.0
            };

            lock_async_mut!(STEPS).set_speed_mod(Some(speed_mod));
        } else {
            lock_async_mut!(STEPS).set_speed_mod(None);
        }

        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn read(sender: Sender<'static, RawMutex, [u8; GRAIN_LEN], 1>) {
    let grain = match lock_async_mut!(STEPS).read_grain().await {
        Err(audio::Error::NoFileLoaded) => [u8::MAX / 2; GRAIN_LEN],
        Err(e) => defmt::panic!("read err: {}", Debug2Format(&e)),
        Ok(v) => v,
    };
    sender.send(grain).await;
}

#[interrupt]
fn TIM4() {
    static CHANNEL: Channel<RawMutex, [u8; GRAIN_LEN], 1> = Channel::new();
    static mut BUFFER: [u8; GRAIN_LEN] = [u8::MAX / 2; GRAIN_LEN];
    static mut INDEX: usize = 0;
    static mut PWM_THIS: Option<InterruptPwm<TIM4>> = None;
    let pwm = PWM_THIS.get_or_insert_with(|| {
        PWM.lock(|pwm| pwm.take().unwrap())
    });

    if *INDEX >= BUFFER.len() {
        *INDEX = 0;
        if CHANNEL.is_full() {
            *BUFFER = CHANNEL.try_receive().unwrap();
        }
        let _ = SPAWNER.lock(|s| {
            s
                .borrow_mut()
                .unwrap()
                .spawn(read(CHANNEL.sender()))
        });
    }

    let sample = BUFFER.get(*INDEX).copied().unwrap_or(u8::MAX / 2);
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

                    let _ = lock_async_mut!(STEPS).buffer(audio::Event::Loop { step: step as u32, length });
                } else if prev_downs.len() > 1 {
                    // init loop stop
                    let _ = lock_async_mut!(STEPS).buffer(audio::Event::Sync);
                } else {
                    // init jump
                    let _ = lock_async_mut!(STEPS).buffer(audio::Event::Hold { step: step as u32 });
                }
            },
            _ => {
                // init sync
                let _ = lock_async_mut!(STEPS).buffer(audio::Event::Sync);
            },
        }
    }
}
