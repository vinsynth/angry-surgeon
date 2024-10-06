#![no_std]
#![no_main]

extern crate alloc;

mod audio;
mod fs;
mod sdio;
use audio::GRAIN_LEN;

use {defmt_rtt as _, panic_probe as _};

use fixed::traits::ToFixed;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::pipe::Pipe;

use embassy_rp::{bind_interrupts, gpio, Peripheral};
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Config,
    FifoJoin,
    InterruptHandler,
    Pio,
    ShiftConfig,
    ShiftDirection,
};

static BYTE_PIPE: Pipe<RawMutex, {GRAIN_LEN * 4}> = Pipe::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 65536;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let config = {
        use embassy_rp::clocks::*;

        let sys_pll = Some(PllConfig {
            refdiv: 2,
            fbdiv: 147,
            post_div1: 4,
            post_div2: 2,
        });
        let usb_pll = Some(PllConfig {
            refdiv: 1,
            fbdiv: 100,
            post_div1: 5,
            post_div2: 5,
        });

        let mut clocks = ClockConfig::crystal(12_000_000);
        clocks.xosc = Some(XoscConfig {
            hz: 12_000_000,
            sys_pll,
            usb_pll,
            delay_multiplier: 128,
        });

        embassy_rp::config::Config::new(clocks)
    };

    let p = embassy_rp::init(config);

    // // pio state machine for SDIO output
    // let mut pio0 = Pio::new(p.PIO0, Irqs);
    // let clock_pin = pio0.common.make_pio_pin(p.PIN_10);
    // let cmd_pin = pio0.common.make_pio_pin(p.PIN_11);
    // let d0_pin = pio0.common.make_pio_pin(p.PIN_12);
    // let d1_pin = pio0.common.make_pio_pin(p.PIN_13);
    // let d2_pin = pio0.common.make_pio_pin(p.PIN_14);
    // let d3_pin = pio0.common.make_pio_pin(p.PIN_15);
    // let cmd_dma_ref = p.DMA_CH1.into_ref();
    // let data_dma_ref = p.DMA_CH2.into_ref();

    // let sdio = sdio::Sdio::new(
    //     pio0,
    //     clock_pin,
    //     cmd_pin,
    //     d0_pin,
    //     d1_pin,
    //     d2_pin,
    //     d3_pin,
    //     cmd_dma_ref,
    //     data_dma_ref,
    // );
    // spawner.must_spawn(handle_sdio(sdio));

    // // block on SDIO init
    // while !BYTE_PIPE.is_full() {
    //     embassy_time::Timer::after_millis(100).await;
    // }

    // // pio state machine for I2S output
    // let mut pio1 = Pio::new(p.PIO1, Irqs);
    // let bit_clock_pin = pio1.common.make_pio_pin(p.PIN_18);
    // let left_right_clock_pin = pio1.common.make_pio_pin(p.PIN_19);
    // let data_pin = pio1.common.make_pio_pin(p.PIN_20);
    // let dma_ref = p.DMA_CH0.into_ref();

    // spawner.must_spawn(handle_i2s(
    //     pio1,
    //     bit_clock_pin,
    //     left_right_clock_pin,
    //     data_pin,
    //     dma_ref,
    // ));

    macro_rules! new_input {
        ( $x:expr ) => {
            gpio::Input::new(
                $x,
                gpio::Pull::Up,
            )
        };
    }

    spawner.must_spawn(handle_clave(
        new_input!(p.PIN_7),
        new_input!(p.PIN_8),
        new_input!(p.PIN_9),
    ));
}

#[embassy_executor::task]
async fn handle_clave(
    mut clave_input: gpio::Input<'static>,
    mut kick_input: gpio::Input<'static>,
    mut snare_input: gpio::Input<'static>,
) {
    use embassy_futures::select::Either3;

    pub struct Debounce<'a> {
        input: gpio::Input<'a>,
    }

    impl<'a> Debounce<'a> {
        pub fn new(input: gpio::Input<'a>) -> Self {
            Self { input }
        }

        pub async fn debounce(&mut self) -> gpio::Level {
            loop {
                let lvl1 = self.input.get_level();
                self.input.wait_for_any_edge().await;
                embassy_time::Timer::after_millis(20).await;
                let lvl2 = self.input.get_level();
                if lvl1 != lvl2 {
                    return lvl2;
                }
            }
        }
    }
    let mut clave_debounce = Debounce::new(clave_input);
    let mut kick_debounce = Debounce::new(kick_input);
    let mut snare_debounce = Debounce::new(snare_input);

    let mut clave_builder: Option<audio::rhythm::ClaveBuilder> = None;
    loop {
        match embassy_futures::select::select3(
            clave_debounce.debounce(),
            kick_debounce.debounce(),
            snare_debounce.debounce(),
        ).await {
            Either3::First(lvl) => {
                if lvl == gpio::Level::High {
                    continue;
                }
                if let Some(clave) = clave_builder.take() {
                    defmt::info!("baking...");
                    let clave = clave.bake();
                    for stroke in clave {
                        defmt::info!("stroke: {}", defmt::Debug2Format(&stroke));
                    }
                } else {
                    clave_builder = Some(audio::rhythm::ClaveBuilder::new());
                }
            },
            Either3::Second(lvl) => {
                if lvl == gpio::Level::High {
                    continue;
                }
                if let Some(clave) = clave_builder.as_mut() {
                    defmt::info!("kick!");
                    clave.push(audio::rhythm::Pulse {
                        onset_type: audio::rhythm::OnsetType::Kick,
                        position: embassy_time::Instant::now().as_millis(),
                    });
                }
            },
            Either3::Third(lvl) => {
                if lvl == gpio::Level::High {
                    continue;
                }
                if let Some(clave) = clave_builder.as_mut() {
                    defmt::info!("snare!");
                    clave.push(audio::rhythm::Pulse {
                        onset_type: audio::rhythm::OnsetType::Snare,
                        position: embassy_time::Instant::now().as_millis(),
                    });
                }
            },
        }
    }
}

#[embassy_executor::task]
async fn handle_i2s(
    mut pio: Pio<'static, PIO1>,
    bit_clock_pin: embassy_rp::pio::Pin<'static, PIO1>,
    left_right_clock_pin: embassy_rp::pio::Pin<'static, PIO1>,
    data_pin: embassy_rp::pio::Pin<'static, PIO1>,
    mut dma_ref: embassy_rp::PeripheralRef<'static, embassy_rp::peripherals::DMA_CH0>,
) {
    let pwm_pio_program = pio_proc::pio_asm!(
        ".side_set 2",
        "    set x 14           side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
        "left_data:",
        "    out pins 1         side 0b00",
        "    jmp x-- left_data  side 0b01",
        "    out pins 1         side 0b10",
        "    set x 14           side 0b11",
        "right_data:",
        "    out pins 1         side 0b10",
        "    jmp x-- right_data side 0b11",
        "    out pins 1         side 0b00",
    );

    let sm0_config = {
        let mut config = Config::default();
        config.use_program(
            &pio.common.load_program(&pwm_pio_program.program),
            &[&bit_clock_pin, &left_right_clock_pin],
        );
        config.set_out_pins(&[&data_pin]);
        const BIT_DEPTH: u32 = 16;
        const CHANNELS: u32 = 2;
        let clock_frequency = audio::SAMPLE_RATE * BIT_DEPTH * CHANNELS;
        config.clock_divider = (110_250_000. / clock_frequency as f64 / 2.).to_fixed();
        config.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        config.fifo_join = FifoJoin::TxOnly;
        config
    };
    pio.sm0.set_config(&sm0_config);
    pio.sm0.set_pin_dirs(
        embassy_rp::pio::Direction::Out,
        &[&data_pin, &left_right_clock_pin, &bit_clock_pin],
    );

    // create two audio buffers (back and front) which will take turns being
    // filled with new audio data and being sent to the pio fifo using dma
    static DMA_BUFFER: StaticCell<[u32; GRAIN_LEN * 2]> = StaticCell::new();
    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; GRAIN_LEN * 2]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(GRAIN_LEN);

    pio.sm0.set_enable(true);
    let tx = pio.sm0.tx();

    loop {
        // trigger transfer of front buffer data to the pio fifo
        // but don't await the returned future, yet
        let dma_future = tx.dma_push(dma_ref.reborrow(), front_buffer);

        // fill back buffer with fresh audio samples before awaiting the dma future
        let mut byte_buffer = [0u8; GRAIN_LEN * 2];
        let mut slice = &mut byte_buffer[..];
        while !slice.is_empty() {
            let n = BYTE_PIPE.read(slice).await;
            slice = &mut slice[n..];
        }

        for (i, word) in back_buffer.iter_mut().enumerate() {
            let mut i16_buffer = [0; 2];

            i16_buffer.copy_from_slice(&byte_buffer[i * 2..][0..2]);
            let left_sample = i16::from_le_bytes(i16_buffer);
            // i16_buffer.copy_from_slice(&byte_buffer[i * 4..][2..4]);
            // let right_sample = i16::from_le_bytes(i16_buffer);
            let right_sample = left_sample;

            *word = (left_sample as u16 as u32) << 16 | (right_sample as u16 as u32);
        }

        // now await the dma future. once the dma finishes, the next buffer needs to be queued
        // within DMA_DEPTH / SAMPLE_RATE = 8 / 48000 seconds = 166us
        dma_future.await;
        core::mem::swap(&mut back_buffer, &mut front_buffer);
    }
}

#[embassy_executor::task]
async fn handle_sdio(
    mut sdio: sdio::Sdio<'static, PIO0>,
) {
    use embedded_io_async::{Read, Seek, SeekFrom};
    sdio.init_card().await.unwrap();

    let sdio_device = fs::SdioDevice::new(sdio);
    let slice = sdio_device.into_stream_slice().await.unwrap();
    let fs_options = embedded_fatfs::FsOptions::new();
    let fs = embedded_fatfs::FileSystem::new(slice, fs_options).await.unwrap();
    let root = fs.root_dir();

    let mut file = root.open_file("amen441mono.wav").await.unwrap();
    let file_length = root.open_meta("amen441mono.wav").await.unwrap().len();

    let rhythm = audio::rhythm::RhythmData::new(&mut file, file_length).await.unwrap();
    defmt::info!("{} pulses: {}", rhythm.pulses.len(), defmt::Debug2Format(&rhythm.pulses[..8]));
    defmt::info!("tempo: {} || step_count: {}", rhythm.tempo, rhythm.step_count);

    defmt::debug!("successfully initialized sdio!");
    file.seek(SeekFrom::Start(44)).await.unwrap();
    loop {
        let mut buffer = [0u8; GRAIN_LEN * 2];
        file.read_exact(&mut buffer).await.unwrap();
        BYTE_PIPE.write_all(&buffer).await;
    }
}
