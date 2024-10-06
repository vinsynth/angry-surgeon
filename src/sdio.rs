//! SDIO driver leveraging the rp2040's PIO module;
//! heavily inspired by carlk3's incredible no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
//! C library, adapted for async Rust

use alloc::format;

use fixed::traits::ToFixed;

use embassy_rp::peripherals::{DMA_CH1, DMA_CH2};
use embassy_rp::PeripheralRef;
use embassy_rp::pio::{
    Config,
    Instance,
    Pin,
    Pio,
    ShiftConfig,
    ShiftDirection,
    StateMachine,
    StatusSource,
};
use micromath::num_traits::ToBytes;

#[derive(Debug)]
pub enum Error {
    DataCrc,
    CmdCrc,
    Timeout,
    UnsupportedFormat,
}

pub enum Response {
    None,
    /// big-endian & MSB-first bits of 48-bit response; doesn't include CRC7 (last) byte
    Bits48(u32),
    /// big-endian & MSB-first bits of CID/CSD response; includes CRC7 (last) byte
    Bits136(u128),
}

#[derive(PartialEq)]
enum ResponseType {
    None,
    R1,
    R1b,
    R2,
    R3,
    R6,
    R7,
}

impl ResponseType {
    fn length(&self) -> u8 {
        match self {
            ResponseType::None => 0,
            ResponseType::R2 => 136,
            _ => 48,
        }
    }
}

pub struct Cmd {
    resp: ResponseType,
    cmd: u8,
    arg: u32,
}

impl Cmd {
    /// GO_IDLE_STATE
    const fn cmd0() -> Self {
        Self {
            resp: ResponseType::None,
            cmd: 0,
            arg: 0,
        }
    }
    /// ALL_SEND_CID
    const fn cmd2() -> Self {
        Self {
            resp: ResponseType::R2,
            cmd: 2,
            arg: 0,
        }
    }
    /// SEND_RELATIVE_ADDR
    const fn cmd3() -> Self {
        Self {
            resp: ResponseType::R6,
            cmd: 3,
            arg: 0,
        }
    }
    /// SWITCH_FUNC
    const fn cmd6(switch: bool, high_speed: bool) -> Self {
        let mut arg: u32 = 0;
        arg |= (switch as u32) << 31;
        arg |= 0xfffff << 4; // don't affect functions 2-6
        arg |= high_speed as u32; // set function 1 (access/bus speed mode)
        Self {
            resp: ResponseType::R1,
            cmd: 6,
            arg,
        }
    }
    /// SELECT/DESELECT_CARD
    const fn cmd7(rca: u16) -> Self {
        let arg: u32 = (rca as u32) << 16;
        Self {
            resp: ResponseType::R1b,
            cmd: 7,
            arg,
        }
    }
    /// SEND_IF_COND
    const fn cmd8(check_pattern: u8) -> Self {
        let mut arg: u32 = 0;
        arg |= 0b0001 << 8;   // host supply voltage 2.7-3.6V
        arg |= check_pattern as u32; // some pattern to check response validity
        Self {
            resp: ResponseType::R1,
            cmd: 8,
            arg,
        }
    }
    /// SEND_CSD
    const fn cmd9(rca: u16) -> Self {
        let arg: u32 = (rca as u32) << 16;
        Self {
            resp: ResponseType::R2,
            cmd: 9,
            arg,
        }
    }
    /// SET_BLOCKLEN
    const fn cmd16(block_len: u32) -> Self {
        let arg: u32 = block_len.to_le();
        Self {
            resp: ResponseType::R1,
            cmd: 16,
            arg,
        }
    }
    /// READ_SINGLE_BLOCK
    const fn cmd17(address: u32) -> Self {
        let arg: u32 = address.to_le();
        Self {
            resp: ResponseType::R1,
            cmd: 17,
            arg,
        }
    }
    /// WRITE_BLOCK
    const fn cmd24(address: u32) -> Self {
        let arg: u32 = address.to_be();
        Self {
            resp: ResponseType::R1,
            cmd: 24,
            arg,
        }
    }
    /// APP_CMD
    const fn cmd55(rca: u16) -> Self {
        let arg: u32 = (rca as u32) << 16;
        Self {
            resp: ResponseType::R7,
            cmd: 55,
            arg,
        }
    }
    /// SET_BUS_WIDTH
    const fn acmd6(four_bit: bool) -> Self {
        let arg: u32 = (four_bit as u32) << 1;
        Self {
            resp: ResponseType::R1,
            cmd: 6,
            arg,
        }
    }
    /// SD_SEND_OP_COND (application-specific command)
    const fn acmd41(supports_sdhc: bool, ocr: u16) -> Self {
        let mut arg: u32 = 0;
        arg |= (supports_sdhc as u32) << 30;
        arg |= (ocr as u32 & 0b111111111) << 15;
        Self {
            resp: ResponseType::R3,
            cmd: 41,
            arg,
        }
    }
    /// SET_CLR_CARD_DETECT
    const fn acmd42(set_cd: bool) -> Self {
        let arg: u32 = set_cd as u32;
        Self {
            resp: ResponseType::R1,
            cmd: 42,
            arg,
        }
    }
    /// SEND_SCR
    const fn acmd51() -> Self {
        Self {
            resp: ResponseType::R1,
            cmd: 51,
            arg: 0,
        }
    }
}

#[cfg(feature = "log-sdio")]
fn print_blocks(start_address: &u32, blocks: &[[u8; 512]]) {
    for (idx, block) in blocks.iter().enumerate() {
        let start_byte = start_address * 512;
        let idx_byte = idx as u32 * 512;

        defmt::trace!(
            "
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}\n{:08x} :: {=[u8]:02x}
            ",
           start_byte + idx_byte + 0x000, block[0x000..0x010],
           start_byte + idx_byte + 0x010, block[0x010..0x020],
           start_byte + idx_byte + 0x020, block[0x020..0x030],
           start_byte + idx_byte + 0x030, block[0x030..0x040],
           start_byte + idx_byte + 0x040, block[0x040..0x050],
           start_byte + idx_byte + 0x050, block[0x050..0x060],
           start_byte + idx_byte + 0x060, block[0x060..0x070],
           start_byte + idx_byte + 0x070, block[0x070..0x080],
           start_byte + idx_byte + 0x080, block[0x080..0x090],
           start_byte + idx_byte + 0x090, block[0x090..0x0a0],
           start_byte + idx_byte + 0x0a0, block[0x0a0..0x0b0],
           start_byte + idx_byte + 0x0b0, block[0x0b0..0x0c0],
           start_byte + idx_byte + 0x0c0, block[0x0c0..0x0d0],
           start_byte + idx_byte + 0x0d0, block[0x0d0..0x0e0],
           start_byte + idx_byte + 0x0e0, block[0x0e0..0x0f0],
           start_byte + idx_byte + 0x0f0, block[0x0f0..0x100],
           start_byte + idx_byte + 0x100, block[0x100..0x110],
           start_byte + idx_byte + 0x110, block[0x110..0x120],
           start_byte + idx_byte + 0x120, block[0x120..0x130],
           start_byte + idx_byte + 0x130, block[0x130..0x140],
           start_byte + idx_byte + 0x140, block[0x140..0x150],
           start_byte + idx_byte + 0x150, block[0x150..0x160],
           start_byte + idx_byte + 0x160, block[0x160..0x170],
           start_byte + idx_byte + 0x170, block[0x170..0x180],
           start_byte + idx_byte + 0x180, block[0x180..0x190],
           start_byte + idx_byte + 0x190, block[0x190..0x1a0],
           start_byte + idx_byte + 0x1a0, block[0x1a0..0x1b0],
           start_byte + idx_byte + 0x1b0, block[0x1b0..0x1c0],
           start_byte + idx_byte + 0x1c0, block[0x1c0..0x1d0],
           start_byte + idx_byte + 0x1d0, block[0x1d0..0x1e0],
           start_byte + idx_byte + 0x1e0, block[0x1e0..0x1f0],
           start_byte + idx_byte + 0x1f0, block[0x1f0..0x200],
        );
    }
}

#[cfg(feature = "log-sdio")]
fn print_words(label: &str, words: &[u32]) {
    defmt::trace!("{}:", label);
    for word in words {
        defmt::trace!(
"{0=31..32}{0=30..31}{0=29..30}{0=28..29}{0=27..28}{0=26..27}{0=25..26}{0=24..25} {0=23..24}{0=22..23}{0=21..22}{0=20..21}{0=19..20}{0=18..19}{0=17..18}{0=16..17} {0=15..16}{0=14..15}{0=13..14}{0=12..13}{0=11..12}{0=10..11}{0=09..10}{0=08..09} {0=07..08}{0=06..07}{0=05..06}{0=04..05}{0=03..04}{0=02..03}{0=01..02}{0=00..01}",
            word,
        );
    }
}

#[cfg(feature = "log-sdio")]
fn print_bytes(label: &str, bytes: &[u8]) {
    defmt::trace!("{}:", label);
    for byte in bytes {
        defmt::trace!(
"{0=7..8}{0=6..7}{0=5..6}{0=4..5}{0=3..4}{0=2..3}{0=1..2}{0=0..1}",
            byte,
        );
    }
}

const CRC7_TABLE: [u8; 256] = [
    0x00, 0x12, 0x24, 0x36, 0x48, 0x5a, 0x6c, 0x7e,	0x90, 0x82, 0xb4, 0xa6, 0xd8, 0xca, 0xfc, 0xee,
    0x32, 0x20, 0x16, 0x04, 0x7a, 0x68, 0x5e, 0x4c,	0xa2, 0xb0, 0x86, 0x94, 0xea, 0xf8, 0xce, 0xdc,
    0x64, 0x76, 0x40, 0x52, 0x2c, 0x3e, 0x08, 0x1a,	0xf4, 0xe6, 0xd0, 0xc2, 0xbc, 0xae, 0x98, 0x8a,
    0x56, 0x44, 0x72, 0x60, 0x1e, 0x0c, 0x3a, 0x28,	0xc6, 0xd4, 0xe2, 0xf0, 0x8e, 0x9c, 0xaa, 0xb8,
    0xc8, 0xda, 0xec, 0xfe, 0x80, 0x92, 0xa4, 0xb6,	0x58, 0x4a, 0x7c, 0x6e, 0x10, 0x02, 0x34, 0x26,
    0xfa, 0xe8, 0xde, 0xcc, 0xb2, 0xa0, 0x96, 0x84,	0x6a, 0x78, 0x4e, 0x5c, 0x22, 0x30, 0x06, 0x14,
    0xac, 0xbe, 0x88, 0x9a, 0xe4, 0xf6, 0xc0, 0xd2,	0x3c, 0x2e, 0x18, 0x0a, 0x74, 0x66, 0x50, 0x42,
    0x9e, 0x8c, 0xba, 0xa8, 0xd6, 0xc4, 0xf2, 0xe0,	0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x70,
    0x82, 0x90, 0xa6, 0xb4, 0xca, 0xd8, 0xee, 0xfc,	0x12, 0x00, 0x36, 0x24, 0x5a, 0x48, 0x7e, 0x6c,
    0xb0, 0xa2, 0x94, 0x86, 0xf8, 0xea, 0xdc, 0xce,	0x20, 0x32, 0x04, 0x16, 0x68, 0x7a, 0x4c, 0x5e,
    0xe6, 0xf4, 0xc2, 0xd0, 0xae, 0xbc, 0x8a, 0x98,	0x76, 0x64, 0x52, 0x40, 0x3e, 0x2c, 0x1a, 0x08,
    0xd4, 0xc6, 0xf0, 0xe2, 0x9c, 0x8e, 0xb8, 0xaa,	0x44, 0x56, 0x60, 0x72, 0x0c, 0x1e, 0x28, 0x3a,
    0x4a, 0x58, 0x6e, 0x7c, 0x02, 0x10, 0x26, 0x34,	0xda, 0xc8, 0xfe, 0xec, 0x92, 0x80, 0xb6, 0xa4,
    0x78, 0x6a, 0x5c, 0x4e, 0x30, 0x22, 0x14, 0x06,	0xe8, 0xfa, 0xcc, 0xde, 0xa0, 0xb2, 0x84, 0x96,
    0x2e, 0x3c, 0x0a, 0x18, 0x66, 0x74, 0x42, 0x50,	0xbe, 0xac, 0x9a, 0x88, 0xf6, 0xe4, 0xd2, 0xc0,
    0x1c, 0x0e, 0x38, 0x2a, 0x54, 0x46, 0x70, 0x62,	0x8c, 0x9e, 0xa8, 0xba, 0xc4, 0xd6, 0xe0, 0xf2,
];

pub struct Sdio<'d, T: Instance> {
    rca: u16,
    cmd_clk_sm: StateMachine<'d, T, 0>,
    data_sm: StateMachine<'d, T, 1>,
    data_rx_cfg: Config<'d, T>,
    data_tx_cfg: Config<'d, T>,
    cmd_dma_ref: PeripheralRef<'d, DMA_CH1>,
    data_dma_ref: PeripheralRef<'d, DMA_CH2>,
    csd: u128,
}

impl<'d, T: Instance> Sdio<'d, T> {
    #[allow(clippy::too_many_arguments)]
    #[allow(non_snake_case)]
    pub fn new(
        mut pio: Pio<'d, T>,
        mut clock_pin: Pin<'d, T>,
        mut cmd_pin: Pin<'d, T>,
        mut d0_pin: Pin<'d, T>,
        mut d1_pin: Pin<'d, T>,
        mut d2_pin: Pin<'d, T>,
        mut d3_pin: Pin<'d, T>,
        cmd_dma_ref: PeripheralRef<'d, DMA_CH1>,
        data_dma_ref: PeripheralRef<'d, DMA_CH2>,
    ) -> Self {
        const CMD_CLK_DIVIDER: f64 = 110_250_000. / 400_000. / 4.;
        const DATA_CLK_DIVIDER: f64 = 110_250_000. / 400_000. / 4.;

        clock_pin.set_pull(embassy_rp::gpio::Pull::Up);
        cmd_pin.set_pull(embassy_rp::gpio::Pull::Up);
        d0_pin.set_pull(embassy_rp::gpio::Pull::Up);
        d1_pin.set_pull(embassy_rp::gpio::Pull::Up);
        d2_pin.set_pull(embassy_rp::gpio::Pull::Up);
        d3_pin.set_pull(embassy_rp::gpio::Pull::Up);

        clock_pin.set_input_sync_bypass(true);
        cmd_pin.set_input_sync_bypass(true);
        d0_pin.set_input_sync_bypass(true);
        d1_pin.set_input_sync_bypass(true);
        d2_pin.set_input_sync_bypass(true);
        d3_pin.set_input_sync_bypass(true);

        clock_pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);
        cmd_pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);
        d0_pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);
        d1_pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);
        d2_pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);
        d3_pin.set_slew_rate(embassy_rp::gpio::SlewRate::Fast);

        let cmd_clk_cfg = {
            let mut config = Config::default();
            let cmd_clk_pgrm = pio.common.load_program(&pio_proc::pio_file!(
                "src/sdio.pio",
                select_program("cmd_clk"),
            ).program);
            config.use_program(
                &cmd_clk_pgrm,
                &[&clock_pin],
            );
            config.set_in_pins(&[&cmd_pin]);
            config.set_out_pins(&[&cmd_pin]);
            config.set_set_pins(&[&cmd_pin]);
            config.set_jmp_pin(&cmd_pin);
            config.status_sel = StatusSource::TxFifoLevel;
            config.status_n = 2;
            config.shift_in = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            config.shift_out = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            config.clock_divider = CMD_CLK_DIVIDER.to_fixed();
            config
        };

        let data_rx_cfg = {
            let mut config = Config::default();
            let data_rx_pgrm = pio.common.load_program(&pio_proc::pio_file!(
                "src/sdio.pio",
                select_program("data_rx"),
            ).program);
            config.use_program(&data_rx_pgrm, &[]);
            config.set_in_pins(&[&d0_pin]);
            config.shift_in = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            config.shift_out = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            // config.fifo_join = FifoJoin::RxOnly;
            config.clock_divider = DATA_CLK_DIVIDER.to_fixed();
            config
        };

        let data_tx_cfg = {
            let mut config = Config::default();
            let data_tx_pgrm = pio.common.load_program(&pio_proc::pio_file!(
                "src/sdio.pio",
                select_program("data_tx"),
            ).program);
            config.use_program(&data_tx_pgrm, &[]);
            config.set_in_pins(&[&d0_pin]);
            config.set_out_pins(&[&d0_pin, &d1_pin, &d2_pin, &d3_pin]);
            config.set_set_pins(&[&d0_pin, &d1_pin, &d2_pin, &d3_pin]);
            config.shift_in = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: false,
            };
            config.shift_out = ShiftConfig {
                threshold: 32,
                direction: ShiftDirection::Left,
                auto_fill: true,
            };
            config.clock_divider = DATA_CLK_DIVIDER.to_fixed();
            config
        };

        pio.sm0.set_config(&cmd_clk_cfg);
        pio.sm0.set_pin_dirs(
            embassy_rp::pio::Direction::Out,
            &[&clock_pin],
        );
        pio.sm0.set_enable(true);

        Self {
            rca: 0,
            cmd_clk_sm: pio.sm0,
            data_sm: pio.sm1,
            data_rx_cfg,
            data_tx_cfg,
            cmd_dma_ref,
            data_dma_ref,
            csd: 0,
        }
    }

    pub async fn init_card(&mut self) -> Result<(), Error> {
        // // wait for voltage ramp
        // embassy_time::Timer::after_millis(37).await;
        // for _ in 0..(110_250_000 / 400_000 * 74) {
        //     cortex_m::asm::nop();
        // }

        self.send_cmd(Cmd::cmd0()).await?;
        while self.send_cmd(Cmd::cmd8(0x45)).await.is_err() {
            self.send_cmd(Cmd::cmd0()).await?;
            embassy_time::Timer::after_millis(1).await;
        }
        
        let start_instant = embassy_time::Instant::now();

        let mut ready = false;
        let mut ocr = 0;
        while !ready {
            self.send_cmd(Cmd::cmd55(0)).await?;
            let response = match self.send_cmd(Cmd::acmd41(true, ocr)).await? {
                Response::Bits48(data) => data,
                _ => defmt::unreachable!(),
            };
            ready = (response >> 31 & 1) == 1;
            ocr = ((response >> 15) & 0b111111111) as u16;

            if ready && (response >> 30) & 1 != 1 { // card is SDSC
                return Err(Error::UnsupportedFormat);
            }
            if embassy_time::Instant::now().duration_since(start_instant).as_millis() > 100 {
                return Err(Error::Timeout);
            }

            embassy_time::Timer::after_millis(10).await;
        }

        self.send_cmd(Cmd::cmd2()).await?;

        self.rca = loop {
            let response = match self.send_cmd(Cmd::cmd3()).await? {
                Response::Bits48(data) => data,
                _ => defmt::unreachable!(),
            };
            let rca = ((response >> 16) & 0xffff) as u16;
            if (response >> 9) & 1 == 1 && rca != 0 { // card is ready
                break rca;
            }
            embassy_time::Timer::after_millis(10).await;
        };

        self.csd = match self.send_cmd(Cmd::cmd9(self.rca)).await? {
            Response::Bits136(data) => data,
            _ => defmt::unreachable!(),
        };

        self.send_cmd(Cmd::cmd7(self.rca)).await?;

        self.send_cmd(Cmd::cmd55(self.rca)).await?;
        self.send_cmd(Cmd::acmd42(false)).await?;

        self.send_cmd(Cmd::cmd55(self.rca)).await?;
        self.send_cmd(Cmd::acmd6(true)).await?;

        self.send_cmd(Cmd::cmd16(512)).await?;

        // FIXME?: support high speed mode
        // >> 50 MHz clock speed impossible if PIO controlling clock speed at 4 cycles per period
        // >> CMD6 switch to high speed mode creates unstable input even without changing clock speed
        //    >> due to reads on falling edge? (docs seem to suggest rising edge)
        // if (csd >> 94) & 1 != 1 { // card dosen't support CMD6
        //     return Err(Error::UnsupportedFormat);
        // }

        // if (rx_buffer[3] >> 17) & 1 != 1 { // high speed function not supported
        //     return Err(Error::UnsupportedFormat);
        // }

        // let mut rx_buffer = [0; 18]; // 512 bits data + 64 bits checksum
        // self.prepare_read(rx_buffer.len() as u32);
        // self.send_cmd(Cmd::cmd6(true, true)).await?;
        // self.read_data(&mut rx_buffer).await?;
        // print_words("switch function status", &rx_buffer);
        
        // // wait for at least 8 clock cycles after read to change clock speed
        // for _ in 0..(110_250_000 / 400_000 * 64) {
        //     cortex_m::asm::nop();
        // }

        const CLOCK_DIVIDER: f64 = 110_250_000. / 25_000_000. / 4.;
        self.cmd_clk_sm.set_clock_divider(CLOCK_DIVIDER.to_fixed());
        self.data_rx_cfg.clock_divider = CLOCK_DIVIDER.to_fixed();

        Ok(())
    }

    pub fn block_count(&self) -> u64 {
        let c_size = ((self.csd >> 48) & 0x3F_FFFF) as u64; // 22 bits
        (c_size + 1) * 1024
    }

    pub async fn read_block(
        &mut self,
        block_address: u32,
        buffer: &mut aligned::Aligned<aligned::A4, [u8; 512]>,
    ) -> Result<(), Error> {
        let mut rx_buffer = [0u32; 130]; // 512 bytes data + 64 bits checksum

        self.prepare_read(rx_buffer.len() as u32 + 2);
        self.send_cmd(Cmd::cmd17(block_address)).await?;
        self.read_data(&mut rx_buffer).await?;

        // [u32] -> [u8]
        for i in 0..128 {
            buffer[4 * i..][..4].copy_from_slice(&rx_buffer[i].to_be_bytes());
        }
        #[cfg(feature = "log-sdio")]
        print_blocks(&block_address, &[**buffer]);

        Ok(())
    }

    pub async fn write_block(
        &mut self,
        block_address: u32,
        buffer: &aligned::Aligned<aligned::A4, [u8; 512]>,
    ) -> Result<(), Error> {
        // [u8] -> [u32]
        let (_, data, _) = unsafe { buffer.align_to::<u32>() };

        // crc16 over interleaved nibbles
        let mut crc = 0u64;
        for word in data {
            let byte_in = *word as u64;
            let mut data_out = crc >> 32;
            crc <<= 32;

            data_out ^= data_out >> 16;
            data_out ^= byte_in >> 16;
            
            let xorred = data_out ^ byte_in;
            crc ^= xorred;
            crc ^= xorred << (5 * 4);
            crc ^= xorred << (12 * 4);
        }

        let mut tx_buffer = [0u32; 132];
        tx_buffer[0] = 0xffff_fff0;          // start token
        tx_buffer[1..=128].copy_from_slice(data);
        tx_buffer[129] = (crc >> 32) as u32; // top 32 bytes CRC16
        tx_buffer[130] = crc as u32;         // bottom 32 bytes CRC16
        tx_buffer[131] = 0xffff_ffff;        // end token

        self.send_cmd(Cmd::cmd24(block_address)).await?;
        self.write_data(&mut tx_buffer).await?;

        Ok(())
    }

    /// assumes 32-bit words
    fn prepare_write(&mut self, word_count: u32) {
        self.data_sm.set_enable(false);
        self.data_sm.set_config(&self.data_tx_cfg);
        unsafe {
            embassy_rp::pio::instr::set_x(
                &mut self.data_sm,
                word_count * 8 - 1 // nibble count of tx_buffer minus 1
            );
            embassy_rp::pio::instr::set_y(
                &mut self.data_sm,
                32 - 1 // response bit count minus 1
            );
            embassy_rp::pio::instr::set_pindir(
                &mut self.data_sm,
                0b1111,
            );
        }
        self.data_sm.set_enable(true);
    }

    /// assumes 32-bit words
    fn prepare_read(&mut self, word_count: u32) {
        self.data_sm.set_enable(false);
        self.data_sm.clear_fifos();
        self.data_sm.set_config(&self.data_rx_cfg);
        unsafe {
            embassy_rp::pio::instr::set_y(
                &mut self.data_sm,
                word_count * 8 - 1 // nibble count of rx_buffer minus 1
            );
        }
        self.data_sm.set_enable(true);
    }

    async fn write_data(&mut self, tx_buffer: &mut [u32]) -> Result<(), Error> {
        let tx_future = self.data_sm.tx().dma_push(
            self.data_dma_ref.reborrow(),
            tx_buffer,
        );
        embassy_time::with_timeout(
            embassy_time::Duration::from_millis(100),
            tx_future,
        ).await.map_err(|_| Error::Timeout)?;

        let mut rx_buffer = [0u32];
        let rx_future = self.data_sm.rx().dma_pull(
            self.data_dma_ref.reborrow(),
            &mut rx_buffer,
        );
        embassy_time::with_timeout(
            embassy_time::Duration::from_millis(100),
            rx_future,
        ).await.map_err(|_| Error::Timeout)?;

        // FIXME: analyze response for crc(?), errors

        Ok(())
    }

    async fn read_data(&mut self, rx_buffer: &mut [u32]) -> Result<(), Error> {
        let future = self.data_sm.rx().dma_pull(
            self.data_dma_ref.reborrow(),
            rx_buffer,
        );
        embassy_time::with_timeout(
            embassy_time::Duration::from_millis(100),
            future,
        ).await.map_err(|_| Error::Timeout)?;

        // crc16 over interleaved nibbles
        let mut crc = 0u64;
        let (data, rx_crc) = rx_buffer.split_last_chunk::<2>().unwrap();
        for word in data {
            let byte_in = *word as u64;
            let mut data_out = crc >> 32;
            crc <<= 32;

            data_out ^= data_out >> 16;
            data_out ^= byte_in >> 16;
            
            let xorred = data_out ^ byte_in;
            crc ^= xorred;
            crc ^= xorred << (5 * 4);
            crc ^= xorred << (12 * 4);
        }

        if rx_crc[0] != (crc >> 32) as u32
            || rx_crc[1] != crc as u32
        {
            return Err(Error::DataCrc);
        }

        Ok(())
    }

    async fn send_cmd(&mut self, cmd: Cmd) -> Result<Response, Error> {
        static mut LAST_CMD: u8 = 0;

        let mut packet: u64 = 0;
        packet |= 0 << 47;
        packet |= 1 << 46;
        packet |= (cmd.cmd as u64) << 40;
        packet |= (cmd.arg as u64) << 8;
        packet |= 1;

        let mut crc = 0u8;
        for byte in &packet.to_be_bytes()[2..=6] {
            crc = CRC7_TABLE[(crc ^ byte) as usize];
        }
        packet |= crc as u64;

        let mut word0: u32 = 0;
        word0 |= (47u8 as u32) << 24;
        word0 |= (packet >> 24) as u32 & 0xffffff;
        let mut word1: u32 = 0;
        word1 |= ((packet) as u32 & 0xffffff) << 8;
        word1 |= (cmd.resp.length()).saturating_sub(1) as u32;

        self.cmd_clk_sm.clear_fifos();

        let tx_buffer = [word0, word1];
        self.cmd_clk_sm.tx().dma_push(self.cmd_dma_ref.reborrow(), &tx_buffer).await;
        let cmd_name = if unsafe { LAST_CMD == 55 } {
            format!("ACMD{}", cmd.cmd)
        } else {
            format!("CMD{}", cmd.cmd)
        };
        #[cfg(feature = "log-sdio")]
        print_words(&format!("sent {}", cmd_name), &tx_buffer);

        let response = match cmd.resp.length() {
            48 => {
                let mut rx_buffer = [0u32; 2];
                self.cmd_clk_sm.rx().dma_pull(self.cmd_dma_ref.reborrow(), &mut rx_buffer).await;
                #[cfg(feature = "log-sdio")]
                print_words(&format!("response to {}", cmd_name), &rx_buffer);

                let mut arg = 0;
                arg |= (rx_buffer[0] & 0xffffff) << 8;
                arg |= (rx_buffer[1] >> 8) & 0xff;

                let mut crc = 0u8;
                crc = CRC7_TABLE[(crc ^ (((rx_buffer[0] >> 24) & 0xff) as u8)) as usize];
                for byte in &arg.to_be_bytes() {
                    crc = CRC7_TABLE[(crc ^ byte) as usize];
                }

                if cmd.resp != ResponseType::R3
                    && rx_buffer[1] & 0b1111_1110 != crc as u32
                {
                    return Err(Error::CmdCrc);
                }

                Response::Bits48(arg)
            },
            136 => {
                let mut rx_buffer = [0u32; 5];
                self.cmd_clk_sm.rx().dma_pull(self.cmd_dma_ref.reborrow(), &mut rx_buffer).await;
                #[cfg(feature = "log-sdio")]
                print_words(&format!("response to {}", cmd_name), &rx_buffer);

                let mut arg_bytes = [0; 16];
                let mut crc = 0u8;
                rx_buffer
                    .iter()
                    .flat_map(|w| w.to_be_bytes())
                    .skip(1)
                    .take(15)
                    .zip(arg_bytes.iter_mut())
                    .for_each(|(rx_byte, arg_byte)| {
                        crc = CRC7_TABLE[(crc ^ rx_byte) as usize];

                        *arg_byte = rx_byte;
                    });

                if rx_buffer[4] & 0b1111_1110 != crc as u32 {
                    return Err(Error::CmdCrc);
                }
                arg_bytes[15] = crc | 1;

                let arg = u128::from_be_bytes(arg_bytes);

                Response::Bits136(arg)
            },
            _ => Response::None,
        };

        unsafe { LAST_CMD = cmd.cmd };
        Ok(response)
    }
}
