#![no_std]
#![no_main]

mod audio;
mod fs;
mod pwm;
mod utils;

use audio::fx::Bitcrush;
use audio::steps::{Command, Event, Steps};
use fs::SdioCard;
use pwm::RingBufferedPwm;
use utils::AsyncMutex;

use defmt_rtt as _;
use panic_probe as _;

extern crate alloc;
use alloc::vec::Vec;
use alloc::string::String;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::{PubSubBehavior, PubSubChannel};
use embassy_time::Instant;

use embassy_stm32::{gpio, interrupt, sdmmc};
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::peripherals::{ADC1, TIM4};
use embassy_stm32::time::Hertz;

const PAD_COUNT: usize = 14;
const GRAIN_LEN: usize = 256;
const ADC_MAX: u16 = 4095;

#[derive(Copy, Clone)]
enum LoggedMxEvent {
    Up(Instant),
    Down(Instant),
}

#[derive(Clone, PartialEq)]
enum MxEvent {
    Down(u8),
    Up(u8),
}

static ADC: AsyncMutex<Option<embassy_stm32::adc::Adc<ADC1>>> = AsyncMutex::new(None);
static POT_PIN: AsyncMutex<Option<embassy_stm32::peripherals::PA4>> = AsyncMutex::new(None);
static BITCRUSH: AsyncMutex<Bitcrush> = AsyncMutex::new(Bitcrush::new());

static mut MX_INS: Option<[gpio::Input; 4]> = None;
static mut MX_OUTS: Option<[gpio::Output; 4]> = None;

static mut TIMER: Option<embassy_stm32::timer::low_level::Timer<embassy_stm32::peripherals::TIM3>> = None;

static COMMAND_CHANNEL: Channel<RawMutex, Command, 1> = Channel::new();
static MX_CHANNEL: PubSubChannel<RawMutex, MxEvent, 1, 2, 4> = PubSubChannel::new();

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
    while let Err(e) = sdmmc.init_card(Hertz::mhz(24)).await {
        if err != Some(e) {
            defmt::info!("waiting for card error, retrying: {:?}", e);
            err = Some(e);
        }
    }
    defmt::info!("clk: {}", sdmmc.clock());

    let card = SdioCard::new(sdmmc);
    let slice = crate::expect!(
        card.into_stream_slice().await,
        "failed to init stream slice",
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
    let mut pwm = RingBufferedPwm::empty(
        p.TIM4,
        Hertz::khz(32),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let max_duty = pwm.get_max_duty() as u16;
    let dma_buf = cortex_m::singleton!(: [u16; GRAIN_LEN] = [0; GRAIN_LEN]).unwrap();
    pwm.start_ch1(
        embassy_stm32::timer::simple_pwm::PwmPin::new_ch1(p.PB6, gpio::OutputType::PushPull),
        p.DMA1_CH0,
        dma_buf,
    );
    spawner.must_spawn(pwm_out(pwm));

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

    let mx_inputs = [
        new_input!(p.PB3),       
        new_input!(p.PB7),
        new_input!(p.PB8),
        new_input!(p.PB9),
    ];
    unsafe { MX_INS.replace(mx_inputs) };
    let mx_outputs = [
        new_output!(p.PA15),
        new_output!(p.PA12),
        new_output!(p.PA11),
        new_output!(p.PA10),        
    ];
    unsafe { MX_OUTS.replace(mx_outputs) };

    let timer = embassy_stm32::timer::low_level::Timer::new(p.TIM3);
    timer.set_frequency(Hertz(1000));
    timer.set_counting_mode(embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp);
    timer.start();
    timer.enable_update_interrupt(true);

    unsafe { TIMER.replace(timer) };
    unsafe { cortex_m::peripheral::NVIC::unmask(embassy_stm32::interrupt::TIM3) };

    spawner.must_spawn(handle_pads());

    defmt::info!("init adc...");
    let adc = embassy_stm32::adc::Adc::new(p.ADC1);
    ADC.lock().await.replace(adc);
    POT_PIN.lock().await.replace(p.PA4);

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
            Command::Paths { sender } => {
                sender.send(steps.paths()).await;
                defmt::info!("sent paths!");
            },
            Command::ReadCut { sender } => {
                let mut grain = [0; GRAIN_LEN];
                match steps.read_cut(max_duty, &mut grain).await {
                    Err(StreamSliceError::Other(Error::NotFound)) => (),
                    Err(e) => defmt::panic!("failed to read cut: {}", defmt::Debug2Format(&e)),
                    _ => (),
                };
                sender.send(grain).await;
            },
        }
    }
}

#[embassy_executor::task]
async fn pwm_out(
    mut pwm: RingBufferedPwm<'static, TIM4, u16>,
) {
    let grain_channel = cortex_m::singleton!(
        : Channel<RawMutex, [u16; GRAIN_LEN], 1> = Channel::new()
    ).unwrap();
    let mut buffer = [0; GRAIN_LEN];

    loop {
        COMMAND_CHANNEL.send(Command::ReadCut { sender: grain_channel.sender() }).await;
        crate::expect!(
            pwm.write(
                embassy_stm32::timer::Channel::Ch1,
                &mut buffer,
            ).await,
            "failed to write pwm",
        );
        buffer = crate::expect!(
            grain_channel.try_receive(),
            "file read too slow :(",
        );
    }
}

#[embassy_executor::task]
async fn handle_pads() {
    #[derive(PartialEq)]
    enum Switch {
        None,
        PushFile { pad_index: Option<usize>, filepaths: Vec<String> },
        PushToSequence,
    }
    let mut switch = Switch::None;
    let mut subscriber = MX_CHANNEL.dyn_subscriber().unwrap();
    let mut prev_downs = Vec::new();

    let paths_channel = cortex_m::singleton!(: Channel<RawMutex, Vec<String>, 1> = Channel::new()).unwrap();

    loop {
        let event = subscriber.next_message_pure().await;
        let mut downs = prev_downs.clone();

        match event {
            MxEvent::Down(14) => {
                let _ = COMMAND_CHANNEL.send(Command::Paths { sender: paths_channel.sender() }).await;
                let filepaths = paths_channel.receive().await;

                switch = Switch::PushFile { pad_index: None, filepaths };
            },
            MxEvent::Down(15) => switch = Switch::PushToSequence,
            MxEvent::Down(idx) => {
                downs.push(idx);

                match switch {
                    Switch::None => handle_step_sequence(prev_downs, &downs).await,
                    Switch::PushFile { ref mut pad_index, ref filepaths } => {
                        let file_index = (
                            lock_async_mut!(ADC).read(lock_async_mut!(POT_PIN)) as f32
                                / (ADC_MAX + 1) as f32
                                * filepaths.len() as f32
                        ) as usize;
                        let filepath = filepaths[file_index].clone();

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
                    },
                    Switch::PushToSequence => {
                        if let Some(pad_index) = downs.last().map(|i| *i as usize) {
                            let _ = COMMAND_CHANNEL.send(Command::PushToSequence { pad_index }).await;
                        }
                    }
                }
            },
            MxEvent::Up(14) => {
                let _ = COMMAND_CHANNEL.send(Command::BakeCuts).await;

                switch = Switch::None;
            },
            MxEvent::Up(15) => {
                let _ = COMMAND_CHANNEL.send(Command::BakeSequence).await;

                switch = Switch::None;
            },
            MxEvent::Up(idx) => {
                downs.retain(|&i| i != idx);

                if switch == Switch::None {
                    handle_step_sequence(prev_downs, &downs).await;
                }
            },
        }
        prev_downs = downs;
    }
}

async fn handle_step_sequence(
    prev_downs: Vec<u8>,
    downs: &Vec<u8>,
) {
    static COUNT_CHANNEL: Channel<RawMutex, usize, 1> = Channel::new();

    if prev_downs != *downs {
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

#[interrupt]
fn TIM3() {
    static mut TIMER_THIS: Option<embassy_stm32::timer::low_level::Timer<embassy_stm32::peripherals::TIM3>> = None;
    static mut MX_INS_THIS: Option<[gpio::Input; 4]> = None;
    static mut MX_OUTS_THIS: Option<[gpio::Output; 4]> = None;

    static mut OUT_IDX: usize = 0;
    static mut EVENT_LOG: [LoggedMxEvent; 16] = [LoggedMxEvent::Up(Instant::MIN); 16];

    let timer = TIMER_THIS.get_or_insert_with(|| {
        unsafe { TIMER.take().unwrap() }
    });
    let mx_ins = MX_INS_THIS.get_or_insert_with(|| {
        unsafe { MX_INS.take().unwrap() }
    });
    let mx_outs = MX_OUTS_THIS.get_or_insert_with(|| {
        unsafe { MX_OUTS.take().unwrap() }
    });

    mx_outs[*OUT_IDX].set_low();
    for (in_idx, mx_in) in mx_ins.iter().enumerate() {
        let mx_index = in_idx * 4 + *OUT_IDX;
        if mx_in.is_high() && matches!(EVENT_LOG[mx_index], LoggedMxEvent::Down(t)
            if Instant::now().duration_since(t).as_micros() > 1000
        ) {
            defmt::debug!("{} up!", mx_index);
            EVENT_LOG[mx_index] = LoggedMxEvent::Up(embassy_time::Instant::now());

            MX_CHANNEL.publish_immediate(MxEvent::Up(mx_index as u8));
        } else if mx_in.is_low() && matches!(EVENT_LOG[mx_index], LoggedMxEvent::Up(t)
            if Instant::now().duration_since(t).as_micros() > 1000
        ) {
            defmt::debug!("{} down!", mx_index);
            EVENT_LOG[mx_index] = LoggedMxEvent::Down(embassy_time::Instant::now());

            MX_CHANNEL.publish_immediate(MxEvent::Down(mx_index as u8));
        }
    }
    mx_outs[*OUT_IDX].set_high();
    *OUT_IDX = (*OUT_IDX + 1) % 4;

    timer.clear_update_interrupt();
}
