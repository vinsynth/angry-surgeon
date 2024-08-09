#![no_std]
#![no_main]

mod audio;
mod fs;
mod pwm;
mod utils;

use audio::fx::Bitcrush;
use audio::steps::{Command, Event, Steps};
use embassy_stm32::exti::ExtiInput;
use fs::SdioCard;
use pwm::InterruptPwm;
use utils::AsyncMutex;

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;
use alloc::boxed::Box;
use alloc::string::String;

use core::cell::RefCell;

use cortex_m::peripheral::NVIC;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::CriticalSectionMutex as SyncMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;

use embassy_stm32::{gpio, interrupt, sdmmc};
use embassy_stm32::peripherals::{ADC1, TIM4};
use embassy_stm32::time::Hertz;

const PAD_COUNT: usize = 12;
const ADC_MAX: u16 = 4095;

type RingBuf<T, const CAP: usize> = arraydeque::ArrayDeque<T, CAP, arraydeque::Wrapping>;

static ADC: AsyncMutex<Option<embassy_stm32::adc::Adc<ADC1>>> = AsyncMutex::new(None);
static POT_PIN: AsyncMutex<Option<embassy_stm32::peripherals::PA4>> = AsyncMutex::new(None);
static MTX: AsyncMutex<RefCell<Vec<u8>>> = AsyncMutex::new(RefCell::new(Vec::new()));
static PWM: SyncMutex<RefCell<Option<InterruptPwm<TIM4>>>> = SyncMutex::new(RefCell::new(None));
static BITCRUSH: AsyncMutex<Bitcrush> = AsyncMutex::new(Bitcrush::new());

static COMMAND_CHANNEL: Channel<RawMutex, Command, 1> = Channel::new();

embassy_stm32::bind_interrupts!(struct Irqs {
    SDIO => sdmmc::InterruptHandler<embassy_stm32::peripherals::SDIO>;
});

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("init heap...");
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

    defmt::info!("init sdmmc...");
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
            defmt::info!("waiting for card error, retrying: {:?}", e);
            err = Some(e);
        }
    }

    let card = SdioCard::new(sdmmc);
    let slice = crate::expect!(
        card.into_stream_slice().await,
        "failed to init strea slice",
    );
    let fs_options = embedded_fatfs::FsOptions::new();
    let fs = crate::expect!(
        embedded_fatfs::FileSystem::new(slice, fs_options).await,
        "failed to init filesystem",
    );
    let root = fs.root_dir();

    let led = gpio::Output::new(
        p.PC13,
        gpio::Level::High,
        gpio::Speed::Low,
    );

    let mut steps = crate::expect!(
        Steps::new(led, root).await,
        "failed to init steps",
    );

    defmt::info!("init pwm...");
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

    defmt::info!("init matrix...");
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

    let _ = spawner.spawn(handle_recording(
        ExtiInput::new(p.PA0, p.EXTI0, gpio::Pull::Up)
    ));

    defmt::info!("init adc...");
    let adc = embassy_stm32::adc::Adc::new(p.ADC1);
    ADC.lock().await.replace(adc);
    POT_PIN.lock().await.replace(p.PA4);

    let _ = spawner.spawn(handle_tempo(new_input!(p.PB0)));

    let _ = spawner.spawn(handle_pads(
        new_input!(p.PB14),
        new_input!(p.PB12),
        new_input!(p.PB10),
    ));

    let _ = spawner.spawn(handle_joystick(
        p.PA2,
        p.PA3,
        new_input!(p.PB13),
    ));

    defmt::info!("enter idle!");
    loop {
        // handle steps
        use block_device_adapters::StreamSliceError;
        use embedded_fatfs::Error;

        let command = COMMAND_CHANNEL.receive().await;
        match command {
            Command::SetTempo { tempo } => {
                let _ = steps.set_tempo(tempo);
            },
            Command::RecordEvents => {
                steps.record_events();
                defmt::info!("started recording!");
            },
            Command::BakeRecording => {
                steps.bake_recording();
                defmt::info!("baked recording!");
            },
            Command::PushEvent { event } => {
                match steps.push_event(event).await {
                    Err(StreamSliceError::Other(Error::NotFound)) => (),
                    Err(e) => defmt::panic!("failed to push event: {}", defmt::Debug2Format(&e)),
                    _ => (),
                };
                defmt::info!("pushed event {}!", event);
            },
            Command::PushToOneshots { path, pad_index } => {
                crate::expect!(
                    steps.push_to_oneshots(path, pad_index).await,
                    "failed to push to oneshots",
                );
                defmt::info!("pushed oneshot to pad {}!", pad_index);
            }
            Command::PlayOneshot { pad_index } => {
                crate::expect!(
                    steps.play_oneshot(pad_index).await,
                    "failed to play oneshot",
                );
                defmt::info!("playing oneshot {}!", pad_index);
            }
            Command::PushToCuts { path, step_count, pad_index } => {
                crate::expect!(
                    steps.push_to_cuts(path, step_count, pad_index).await,
                    "failed to push to cuts",
                );
                defmt::info!("pushed {} steps to pad {}!", step_count, pad_index);
            },
            Command::BakeCuts => {
                let _ = steps.bake_cuts();
                defmt::info!("baked cuts!");
            },
            Command::PushToSequence { pad_index } => {
                steps.push_to_sequence(pad_index);
                defmt::info!("pushed pad {} to sequence!", pad_index);
            },
            Command::BakeSequence => {
                match steps.bake_sequence().await {
                    Err(StreamSliceError::Other(Error::NotFound)) => (),
                    Err(e) => defmt::panic!("failed to bake sequence: {}", defmt::Debug2Format(&e)),
                    _ => (),
                }
                defmt::info!("baked sequence!");
            },
            Command::StepCount { sender } => sender.send(steps.step_count().unwrap_or(0)).await,
            Command::Paths { sender } => sender.send(steps.paths()).await,
            Command::ReadCut { sender } => {
                let mut grain = match steps.read_cut().await {
                    Err(StreamSliceError::Other(Error::NotFound)) => Box::new([u8::MAX / 2; 512]),
                    Err(e) => defmt::panic!("failed to read cut: {}", defmt::Debug2Format(&e)),
                    Ok(g) => g,
                };
                let oneshot = match steps.read_oneshot().await {
                    Err(StreamSliceError::Other(Error::NotFound)) => {
                        vec![u8::MAX / 2; grain.len()].into_boxed_slice()
                    },
                    Err(e) => defmt::panic!("failed to read oneshot: {}", defmt::Debug2Format(&e)),
                    Ok(g) => g,
                };
                grain
                    .iter_mut()
                    .zip(oneshot.iter())
                    .for_each(|(g, o)| {
                        *g = ((*g as f32 + *o as f32) / 2.0) as u8;
                    });
                BITCRUSH.lock().await.crush(&mut grain);
                sender.send(grain).await;
            },
        }
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

        for (out_idx, mx_output) in mx_outputs.iter_mut().enumerate() {
            mx_output.set_low();
            for (in_idx, mx_input) in mx_inputs.iter().enumerate() {
                let index = in_idx as u8 * 4 + out_idx as u8;
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
async fn handle_recording (
    mut sw: ExtiInput<'static>,
) {
    loop {
        sw.wait_for_falling_edge().await;
        COMMAND_CHANNEL.send(Command::RecordEvents).await;
        Timer::after_millis(17).await;
        sw.wait_for_rising_edge().await;
        Timer::after_millis(17).await;

        sw.wait_for_falling_edge().await;
        COMMAND_CHANNEL.send(Command::BakeRecording).await;
        Timer::after_millis(17).await;
        sw.wait_for_rising_edge().await;
        Timer::after_millis(17).await;
    }
}

#[embassy_executor::task]
async fn handle_tempo (
    sw: gpio::Input<'static>,
) {
    let mut adc_buf: RingBuf<u16, 16> = RingBuf::new();

    loop {
        if sw.is_low() {
        let length = adc_buf.len();
        adc_buf.push_back(lock_async_mut!(ADC).read(lock_async_mut!(POT_PIN)));
            let wma = adc_buf
                .iter()
                .enumerate()
                .fold(0.0, |acc, (i, &v)| {
                    acc + v as f32 * (i + 1) as f32
                }) / (length * (length + 1) / 2) as f32;
            let tempo = wma / ADC_MAX as f32 * 540.0 + 60.0;
            let _ = COMMAND_CHANNEL.send(Command::SetTempo { tempo }).await;
        } else {
            adc_buf.clear();
        }
        embassy_time::Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn handle_pads (
    push_file_sw: gpio::Input<'static>,
    seq_sw: gpio::Input<'static>,
    play_oneshot_sw: gpio::Input<'static>,
) {
    enum Switch {
        None,
        PushFile { pad_index: Option<usize> },
        PushToSequence,
        PlayOneshot { played: bool },
    }
    let mut switch = Switch::None;

    static PATHS_CHANNEL: Channel<RawMutex, Vec<String>, 1> = Channel::new();
    let mut filepaths: Option<Vec<String>> = None;
    let mut prev_filepath = None;

    let mut adc_buf: RingBuf<u16, 16> = RingBuf::new();
    let filepath = |adc_buf: &mut RingBuf<u16, 16>, adc: u16, filepaths: Vec<String>| {
        adc_buf.push_back(adc);
        let length = adc_buf.len();
        let wma = adc_buf
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, &v)| {
                acc + v as f32 * (i + 1) as f32
            }) / (length * (length + 1) / 2) as f32;
        let file_index = ((wma / ADC_MAX as f32).clamp(0.0, 1.0) * filepaths.len() as f32) as usize;
        filepaths[file_index].clone()
    };

    let mut prev_downs = Vec::new();

    loop {
        let downs = MTX.lock().await.borrow().clone();

        match switch {
            Switch::None => {
                if push_file_sw.is_low() {
                    switch = Switch::PushFile { pad_index: None };
                } else if seq_sw.is_low() {
                    switch = Switch::PushToSequence;
                } else if play_oneshot_sw.is_low() {
                    switch = Switch::PlayOneshot { played: false };
                } else {
                    handle_step_sequence(prev_downs, downs.clone()).await;
                }
            },
            Switch::PushFile { ref mut pad_index } => {
                if push_file_sw.is_high() {
                    if let Some(pad_index) = pad_index.take() {
                        if filepaths.is_none() {
                            let _ = COMMAND_CHANNEL.send(Command::Paths { sender: PATHS_CHANNEL.sender() }).await;
                            filepaths = Some(PATHS_CHANNEL.receive().await);
                        }
                        let filepath = filepath(&mut adc_buf, lock_async_mut!(ADC).read(lock_async_mut!(POT_PIN)), filepaths.clone().unwrap());

                        let _ = COMMAND_CHANNEL.send(Command::PushToOneshots { path: filepath, pad_index }).await;
                    }
                    switch = Switch::None;
                    prev_filepath = None;
                    adc_buf.clear();

                    let _ = COMMAND_CHANNEL.send(Command::BakeCuts).await;
                } else {
                    if filepaths.is_none() {
                        let _ = COMMAND_CHANNEL.send(Command::Paths { sender: PATHS_CHANNEL.sender() }).await;
                        filepaths = Some(PATHS_CHANNEL.receive().await);
                    }
                    let filepath = filepath(&mut adc_buf, lock_async_mut!(ADC).read(lock_async_mut!(POT_PIN)), filepaths.clone().unwrap());

                    if prev_filepath.clone().is_none() || prev_filepath.clone().is_some_and(|p| p != filepath) {
                        defmt::info!("targeting filepath: {}", defmt::Debug2Format(&filepath));
                        prev_filepath = Some(filepath.clone());
                    }

                    if downs.len() > prev_downs.len() {
                        if let Some(down) = downs.last().map(|i| *i as usize) {
                            if let Some(pad_index) = pad_index.take() {
                                // assign steps of file to pad
                                let step_count = down + 1;
                                let _ = COMMAND_CHANNEL.send(Command::PushToCuts { path: filepath, step_count, pad_index }).await;
                            } else {
                                // store pad_index for command on next down
                                *pad_index = Some(down);
                            }
                        }
                    }
                }
            },
            Switch::PushToSequence => {
                if seq_sw.is_high() {
                    switch = Switch::None;
                    
                    let _ = COMMAND_CHANNEL.send(Command::BakeSequence).await;
                } else if downs.len() > prev_downs.len() {
                    if let Some(pad_index) = downs.last().map(|i| *i as usize) {
                        let _ = COMMAND_CHANNEL.send(Command::PushToSequence { pad_index }).await;
                    }
                }
            },
            Switch::PlayOneshot { ref mut played }=> {
                if play_oneshot_sw.is_high() {
                    if !*played {
                        let _ = COMMAND_CHANNEL.send(Command::PlayOneshot { pad_index: None }).await;
                    }
                    switch = Switch::None;
                } else if downs.len() > prev_downs.len() {
                    if let Some(down) = downs.last().map(|i| *i as usize) {
                        *played = true;
                        let _ = COMMAND_CHANNEL.send(Command::PlayOneshot { pad_index: Some(down) }).await;
                    }
                }
            },
        }
        prev_downs = downs;
        Timer::after_millis(1).await;
    }
}

async fn handle_step_sequence(
    prev_downs: Vec<u8>,
    downs: Vec<u8>,
) {
    static COUNT_CHANNEL: Channel<RawMutex, usize, 1> = Channel::new();

    if prev_downs != downs {
        let _ = COMMAND_CHANNEL.send(Command::StepCount { sender: COUNT_CHANNEL.sender() }).await;
        let step_count = COUNT_CHANNEL.receive().await;

        match downs.first() {
            Some(&step) if step_count > step as usize => {
                if downs.len() > 1 {
                    // init loop start
                    let numerator = downs
                        .iter()
                        .skip(1)
                        .map(|d| d.checked_sub(step + 1).unwrap_or(d + PAD_COUNT as u8 - 1 - step))
                        .fold(0u32, |acc, d| acc | 1 << d);
                    let length = utils::Fraction::new(numerator, 16);

                    let _ = COMMAND_CHANNEL.send(Command::PushEvent {
                        event: Event::Loop { step: step as usize, length }
                    }).await;
                } else if prev_downs.len() > 1 {
                    // init loop stop
                    let _ = COMMAND_CHANNEL.send(Command::PushEvent {
                        event: Event::Sync
                    }).await;
                } else {
                    // init jump
                    let _ = COMMAND_CHANNEL.send(Command::PushEvent {
                        event: Event::Hold { step: step as usize }
                    }).await;
                }
            },
            _ => {
                // init sync
                let _ = COMMAND_CHANNEL.send(Command::PushEvent {
                    event: Event::Sync
                }).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn handle_joystick(
    mut x_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    mut y_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    joy_sw: gpio::Input<'static>,
) {
    let mut buf: RingBuf<(u16, u16), 16> = RingBuf::new();
    loop {
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
                    x / (length * (length + 1) / 2) as f32 / ADC_MAX as f32,
                    y / (length * (length + 1) / 2) as f32 / ADC_MAX as f32,
                )
            };
            let bit_depth = (x + 1.0) * (x - 1.0) + 1.0;
            let rate = (y + 1.0) * (y - 1.0) + 1.0;
            BITCRUSH.lock().await.set_bit_depth(bit_depth);
            BITCRUSH.lock().await.set_rate(rate);
        }

        Timer::after_millis(1).await;
    }
}

#[interrupt]
fn TIM4() {
    static CHANNEL: Channel<RawMutex, Box<[u8]>, 1> = Channel::new();
    static mut BUFFER: Option<Box<[u8]>> = None;
    static mut INDEX: usize = 0;
    static mut PWM_THIS: Option<InterruptPwm<TIM4>> = None;
    let buffer = BUFFER.get_or_insert_with(|| Box::new([]));
    let pwm = PWM_THIS.get_or_insert_with(|| {
        PWM.lock(|pwm| pwm.take().unwrap())
    });

    if *INDEX >= buffer.len() {
        *INDEX = 0;
        if CHANNEL.is_full() {
            *buffer = CHANNEL.try_receive().unwrap();
        }
        let sender = CHANNEL.sender();
        let _ = COMMAND_CHANNEL.try_send(Command::ReadCut { sender });
    }

    let sample = buffer.get(*INDEX).copied().unwrap_or(u8::MAX / 2);
    *INDEX += 1;

    let duty = (sample as f32 / u8::MAX as f32 * pwm.get_max_duty() as f32) as u32;

    if let Err(e) = pwm.set_duty(embassy_stm32::timer::Channel::Ch3, duty) {
        defmt::panic!("failed to set duty: {}", defmt::Debug2Format(&e));
    }

    pwm.clear_update_interrupt();
}
