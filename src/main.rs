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

type RingBuf<T, const CAP: usize> = arraydeque::ArrayDeque<T, CAP, arraydeque::Wrapping>;

static PWM: SyncMutex<RefCell<Option<InterruptPwm<TIM4>>>> = SyncMutex::new(RefCell::new(None));

static SPAWNER: SyncMutex<RefCell<Option<SendSpawner>>> = SyncMutex::new(RefCell::new(None));
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
    info!("sdmmc clock: {}", sdmmc.clock());

    let card = fs::SdioCard::new(sdmmc);

    let mut vol_mgr = embedded_sdmmc::VolumeManager::new(card, fs::DummyTimesource::default());
    info!("card size is {} bytes",
        vol_mgr.device().num_bytes().await.expect("failed to retrieve card size")
    );

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

    // info!("init adc...");
    // let adc = embassy_stm32::adc::Adc::new(p.ADC1);
    // let joy_sw = gpio::Input::new(p.PB13, gpio::Pull::Up);
    // let _ = spawner.spawn(handle_adc(adc, p.PA2, p.PA3, joy_sw));

    info!("init tapper...");
    let tap_sw = ExtiInput::new(p.PB14, p.EXTI14, gpio::Pull::Up);
    let _ = spawner.spawn(handle_tapper(tap_sw));

    info!("init mode...");
    let mode_sw = ExtiInput::new(p.PA0, p.EXTI0, gpio::Pull::Up);
    let _ = spawner.spawn(handle_mode(mode_sw));

    info!("init matrix...");
    macro_rules! new_mx_output {
        ( $x:expr ) => {
            gpio::Output::new(
                $x,
                gpio::Level::High,
                gpio::Speed::Low,
            )
        };
    }
    macro_rules! new_mx_input {
        ( $x:expr ) => {
            gpio::Input::new(
                $x,
                gpio::Pull::Up,
            )
        };
    }

    let mx_input0 = new_mx_input!(p.PB3);
    let mx_input1 = new_mx_input!(p.PB6);
    let mx_input2 = new_mx_input!(p.PB7);
    let mx_input3 = new_mx_input!(p.PB9);

    let mx_output0 = new_mx_output!(p.PA10);
    let mx_output1 = new_mx_output!(p.PA11);
    let mx_output2 = new_mx_output!(p.PA12);
    let mx_output3 = new_mx_output!(p.PA15);

    let file_sw = new_mx_input!(p.PB12);

    let _ = spawner.spawn(handle_matrix(
        [mx_input0, mx_input1, mx_input2, mx_input3],
        [mx_output0, mx_output1, mx_output2, mx_output3],
        file_sw,
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
        let _ = lock_async_mut!(STEPS).set_running(false).await;
        input.wait_for_falling_edge().await;
        let _ = lock_async_mut!(STEPS).set_running(true).await;
    }
}

#[embassy_executor::task]
async fn handle_tapper(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_falling_edge().await;
        let mut prev = lock_async_mut!(STEPS).tap(None, false).await.unwrap().unwrap();

        while embassy_time::with_timeout(
            embassy_time::Duration::from_secs(2),
            input.wait_for_low()
        ).await.is_ok() {
            prev = lock_async_mut!(STEPS).tap(Some(prev), false).await.unwrap().unwrap();
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
    file_input: gpio::Input<'static>,
) {
    let mut prev_downs = Vec::new();
    loop {
        let step_count = lock_async_ref!(STEPS).step_count().unwrap();
        let mut downs = prev_downs.clone();

        // process matrix
        for (o_idx, mx_output) in mx_outputs.iter_mut().enumerate() {
            mx_output.set_low();
            for (i_idx, mx_input) in mx_inputs.iter().enumerate() {
                let index = i_idx * 4 + o_idx;
                if mx_input.is_high() && downs.contains(&index) {
                    downs.retain(|&v| v != index);
                } else if mx_input.is_low() && !downs.contains(&index) && index < step_count as usize {
                    info!("down: {}", index);
                    downs.push(index);

                    if file_input.is_low() && index < 2 {
                        let file_name = match index {
                            0 => "gentle74.wav",
                            1 => "amen74.wav",
                            _ => defmt::unimplemented!(),
                        };

                        crate::expect!(
                            lock_async_mut!(STEPS).load_file(file_name).await,
                            "failed to load file"
                        );
                        info!("loaded file {}!", file_name);
                    }
                }
            }
            mx_output.set_high();

            Timer::after_millis(1).await;
        }

        // process state
        if prev_downs != downs {
            if let Some(&step) = downs.first() {
                if downs.len() > 1 {
                    // init loop start
                    let numerator = downs
                        .iter()
                        .skip(1)
                        .map(|v| v.checked_sub(step + 1).unwrap_or(v + step_count as usize - 1 - step))
                        .fold(0u32, |acc, v| acc | 1 << v);

                    info!("numerator: {}, downs: {}", numerator, Debug2Format(&downs));
                    let length = utils::Fraction::new(numerator, 16);

                    lock_async_mut!(STEPS).buffer(audio::Event::Loop { step: step as u32, length });
                } else if prev_downs.len() > 1 {
                    // init loop stop
                    info!("init loop stop!");
                    lock_async_mut!(STEPS).buffer(audio::Event::Sync);
                } else {
                    // init jump
                    info!("init jump to {}!", step);
                    lock_async_mut!(STEPS).buffer(audio::Event::Hold { step: step as u32 });
                }
            } else {
                // init sync
                info!("init sync!");
                lock_async_mut!(STEPS).buffer(audio::Event::Sync);
            }
        }

        prev_downs = downs;
    }
}

#[embassy_executor::task]
async fn handle_adc(
    mut adc: embassy_stm32::adc::Adc<'static, ADC1>,
    mut x_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    mut y_pin: impl embassy_stm32::adc::AdcPin<ADC1> + 'static,
    sw: gpio::Input<'static>,
) {
    let mut x: RingBuf<u16, 16> = RingBuf::new();
    loop {
        if sw.is_low() {
            x.push_back(adc.read(&mut x_pin));

            let length = x.len();
            let wma = x
                .iter()
                .enumerate()
                .fold(0.0, |acc, (i, &v)| {
                    acc + v as f32 * (i + 1) as f32
                }) / (length * (length + 1) / 2) as f32;
            let speed = lock_async_ref!(STEPS).speed();
            let speed_mod = if wma > 2047.0 {
                ((wma / 2047.0 - 1.0) * 3.0 + 1.0) * speed
            } else {
                wma / 2047.0 * speed
            };

            lock_async_mut!(STEPS).set_speed_mod(Some(speed_mod));
        } else if !x.is_empty() {
            x = RingBuf::new();
            lock_async_mut!(STEPS).set_speed_mod(None);
        }

        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn read(sender: Sender<'static, RawMutex, [u8; GRAIN_LEN], 1>) {
    let grain = crate::expect!(
        lock_async_mut!(STEPS).read_grain().await,
        "failed to read grain"
    );
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
