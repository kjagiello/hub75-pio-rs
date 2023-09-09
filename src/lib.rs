//! # hub75-pio
//!
//! An **experimental** HUB75 driver for [RP2040](https://www.raspberrypi.com/products/rp2040/).
//! Utilizes Programmable I/O (PIO) units in combination with DMA to achieve high refresh rates,
//! true color depth and zero CPU overhead without sacrificing quality.
//!
//! ## Features
//!
//! - Supports LED matrices up to 64x32 pixels with 1:16 scanline
//! - High refresh rate (approx. 2100 Hz with 24 bit color depth on a 64x32
//!   display)
//! - Does not utilize CPU for clocking out data to the display – all the work is
//!   done by the PIOs and the DMA controller
//! - Uses binary color modulation
//! - Double buffered
//! - Implements [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) traits
//!
//! ## Requirements
//!
//! The current implementation assumes that the following groups of data outputs
//! are assigned to consecutive pins on the RP2040:
//!
//! - R1, G1, B1, R2, G2, B2
//! - ADDRA, ADDRB, ADDRC, ADDRD

#![no_std]
#![feature(generic_const_exprs)]
#![feature(const_for)]

// TODO: Implement the drop trait to release DMA & PIO?
// TODO: organize these
use crate::dma::{Channel, ChannelIndex, ChannelRegs};
use core::convert::TryInto;
use embedded_graphics::prelude::*;
use rp2040_hal::gpio::dynpin::{DynPin, DynPinMode};
use rp2040_hal::gpio::FunctionConfig;
use rp2040_hal::pio::{
    Buffers, PIOBuilder, PIOExt, PinDir, ShiftDirection, StateMachineIndex, UninitStateMachine, PIO,
};

pub mod dma;
pub mod lut;

/// Framebuffer size in bytes
#[doc(hidden)]
pub const fn fb_bytes(w: usize, h: usize, b: usize) -> usize {
    w * h / 2 * b
}

/// Computes an array with number of clock ticks to wait for every n-th color bit
const fn delays<const B: usize>() -> [u32; B] {
    let mut arr = [0; B];
    let mut i = 0;
    while i < arr.len() {
        arr[i] = (1 << i) - 1;
        i += 1;
    }
    arr
}

/// Backing storage for the framebuffers
///
/// ## Memory layout
///
/// The pixel buffer is organized in a way way so that transmiting the content of it requires no
/// manipulation on the PIO.
///
/// PIO reads the pixel buffer one byte at a time from low to high, then shifts out the pixels from
/// left to right.
///
/// ### Pixel tuple
///
/// At the lowest level, every byte in the buffer contains a so called pixel tuple. Within a tuple
/// you will find the nth bit of the R/G/B-componts of two colors. The reason for packing two
/// colors together is the scan rate of the display. The display this library is targetting is has
/// a 1:16 scan rate, which means that when you are addressing a pixel at row 0, you are at the
/// same time addressing a pixel in the same column at row 16.
///
/// The tuple has the following structure:
///
/// ```
/// XXBGRBGR
/// --222111
/// ```
///
/// Currently we are wasting two bits per tuple (byte) – marked as X above.
///
/// ### Buffer structure
///
/// The diagram below attempts to visualize the structure of the buffer for a 64x32 display
/// configured for 24 bit color depth. X, Y are the pixel coordinates on the screen and N stands
/// for the big-endian position of the bit in the color to be displayed.
///
/// ```
///   N           8                                               1             0
///   X   |      63|   |       3        2|       1|       0|      63|   |       0|
/// Y   0 |XXBGRBGR|...|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|...|XXBGRBGR|
///       |XX222111|...|XX222111|XX222111|XX222111|XX222111|XX222111|...|XX222111|
///       |XX000000|...|XX000000|XX000000|XX000000|XX000000|XX000000|...|XX000000|
///       |--------|...|--------|--------|--------|--------|--------|...|--------|
/// Y   1 |XXBGRBGR|...|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|...|XXBGRBGR|
///       |XX222111|...|XX222111|XX222111|XX222111|XX222111|XX222111|...|XX222111|
///       |XX000000|...|XX000000|XX000000|XX000000|XX000000|XX000000|...|XX000000|
///       |--------|...|--------|--------|--------|--------|--------|...|--------|
///       .................................................|.....................|
///       .................................................|.....................|
///       .................................................|.....................|
/// Y  15 |XXBGRBGR|...|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|XXBGRBGR|...|XXBGRBGR|
///       |XX222111|...|XX222111|XX222111|XX222111|XX222111|XX222111|...|XX222111|
///       |XX000000|...|XX000000|XX000000|XX000000|XX000000|XX000000|...|XX000000|
///       |--------|...|--------|--------|--------|--------|--------|...|--------|
/// ```
pub struct DisplayMemory<const W: usize, const H: usize, const B: usize>
where
    [(); fb_bytes(W, H, B)]: Sized,
{
    fbptr: [u32; 1],
    fb0: [u8; fb_bytes(W, H, B)],
    fb1: [u8; fb_bytes(W, H, B)],
    delays: [u32; B],
    delaysptr: [u32; 1],
}

impl<const W: usize, const H: usize, const B: usize> DisplayMemory<W, H, B>
where
    [(); fb_bytes(W, H, B)]: Sized,
{
    pub const fn new() -> Self {
        let fb0 = [0; fb_bytes(W, H, B)];
        let fb1 = [0; fb_bytes(W, H, B)];
        let fbptr: [u32; 1] = [0];
        let delays = delays();
        let delaysptr: [u32; 1] = [0];
        DisplayMemory {
            fbptr,
            fb0,
            fb1,
            delays,
            delaysptr,
        }
    }
}

/// Mapping between GPIO pins and HUB75 pins
pub struct DisplayPins {
    pub r1: DynPin,
    pub g1: DynPin,
    pub b1: DynPin,
    pub r2: DynPin,
    pub g2: DynPin,
    pub b2: DynPin,
    pub clk: DynPin,
    pub addra: DynPin,
    pub addrb: DynPin,
    pub addrc: DynPin,
    pub addrd: DynPin,
    pub lat: DynPin,
    pub oe: DynPin,
}

/// The HUB75 display driver
pub struct Display<'a, CH1, const W: usize, const H: usize, const B: usize, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: ChannelIndex,
    C: RgbColor,
{
    mem: &'static mut DisplayMemory<W, H, B>,
    fb_loop_ch: Channel<CH1>,
    benchmark: bool,
    brightness: u8,
    lut: &'a dyn lut::Lut<B, C>,
}

impl<'a, CH1, const W: usize, const H: usize, const B: usize, C> Display<'a, CH1, W, H, B, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: ChannelIndex,
    C: RgbColor,
{
    /// Creates a new display
    ///
    /// Algo:
    /// 1. Enable output
    /// 2. Clock out the row:
    ///  1. Set rgb pins
    ///  2. Cycle the clock once
    ///  3. Go back to 2 and repeat 63 more times
    /// 4. Disable output
    /// 5. Cycle the latch
    /// 6. Select the row
    ///
    /// # Arguments
    ///
    /// * `pins`: Pins to use for the communication with the display
    /// * `pio_sms`: PIO state machines to be used to drive the display
    /// * `dma_chs`: DMA channels to be used to drive the PIO state machines
    pub fn new<PE, SM0, SM1, SM2, CH0, CH2, CH3>(
        buffer: &'static mut DisplayMemory<W, H, B>,
        mut pins: DisplayPins,
        pio_block: &mut PIO<PE>,
        pio_sms: (
            UninitStateMachine<(PE, SM0)>,
            UninitStateMachine<(PE, SM1)>,
            UninitStateMachine<(PE, SM2)>,
        ),
        dma_chs: (Channel<CH0>, Channel<CH1>, Channel<CH2>, Channel<CH3>),
        benchmark: bool,
        lut: &'a impl lut::Lut<B, C>,
    ) -> Self
    where
        PE: PIOExt + FunctionConfig,
        SM0: StateMachineIndex,
        SM1: StateMachineIndex,
        SM2: StateMachineIndex,
        CH0: ChannelIndex,
        CH2: ChannelIndex,
        CH3: ChannelIndex,
        C: RgbColor,
    {
        let fpins = [
            &mut pins.r1,
            &mut pins.g1,
            &mut pins.b1,
            &mut pins.r2,
            &mut pins.g2,
            &mut pins.b2,
            &mut pins.clk,
            &mut pins.addra,
            &mut pins.addrb,
            &mut pins.addrc,
            &mut pins.addrd,
            &mut pins.lat,
            &mut pins.oe,
        ];
        fpins.map(|pin| pin.try_into_mode(DynPinMode::Function(PE::DYN)).unwrap());

        // Setup PIO SMs
        let (data_sm, row_sm, oe_sm) = pio_sms;

        // Data SM
        let (data_sm, data_sm_tx) = {
            let program_data = pio_proc::pio_asm!(
                ".side_set 1",
                "out isr, 32    side 0b0",
                ".wrap_target",
                "mov x isr      side 0b0",
                // Wait for the row program to set the ADDR pins
                "pixel:",
                "out pins, 8    side 0b0",
                "jmp x-- pixel  side 0b1", // clock out the pixel
                "irq 4          side 0b0", // tell the row program to set the next row
                "wait 1 irq 5   side 0b0",
                ".wrap",
            );
            let installed = pio_block.install(&program_data.program).unwrap();
            let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
                .out_pins(pins.r1.id().num, 6)
                .side_set_pin_base(pins.clk.id().num)
                .clock_divisor_fixed_point(2, 0)
                .out_shift_direction(ShiftDirection::Right)
                .autopull(true)
                .buffers(Buffers::OnlyTx)
                .build(data_sm);
            sm.set_pindirs([
                (pins.r1.id().num, PinDir::Output),
                (pins.g1.id().num, PinDir::Output),
                (pins.b1.id().num, PinDir::Output),
                (pins.r2.id().num, PinDir::Output),
                (pins.g2.id().num, PinDir::Output),
                (pins.b2.id().num, PinDir::Output),
                (pins.clk.id().num, PinDir::Output),
            ]);
            // Configure the width of the screen
            tx.write((W - 1).try_into().unwrap());
            (sm, tx)
        };

        let row_sm = {
            // Row program
            let program_data = pio_proc::pio_asm!(
                ".side_set 1",
                "pull           side 0b0", // Pull the height / 2 into OSR
                "out isr, 32    side 0b0", // and move it to OSR
                "pull           side 0b0", // Pull the color depth - 1 into OSR
                ".wrap_target",
                "mov x, isr     side 0b0",
                "addr:",
                "mov pins, ~x   side 0b0", // Set the row address
                "mov y, osr     side 0b0",
                "row:",
                "wait 1 irq 4   side 0b0", // Wait until the data is clocked in
                "nop            side 0b1",
                "irq 6          side 0b1", // Display the latched data
                "irq 5          side 0b0", // Clock in next row
                "wait 1 irq 7   side 0b0", // Wait for the OE cycle to complete
                "jmp y-- row    side 0b0",
                "jmp x-- addr   side 0b0",
                ".wrap",
            );
            let installed = pio_block.install(&program_data.program).unwrap();
            let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
                .out_pins(pins.addra.id().num, 4)
                .side_set_pin_base(pins.lat.id().num)
                .clock_divisor_fixed_point(1, 1)
                .build(row_sm);
            sm.set_pindirs([
                (pins.addra.id().num, PinDir::Output),
                (pins.addrb.id().num, PinDir::Output),
                (pins.addrc.id().num, PinDir::Output),
                (pins.addrd.id().num, PinDir::Output),
                (pins.lat.id().num, PinDir::Output),
            ]);
            // Configure the height of the screen
            tx.write((H / 2 - 1).try_into().unwrap());
            // Configure the color depth
            tx.write((B - 1).try_into().unwrap());
            sm
        };

        let (oe_sm, oe_sm_tx) = {
            // Control the delay using DMA - buffer with 8 bytes specifying the length of the delays
            // Delay program (controls OE)
            let program_data = pio_proc::pio_asm!(
                ".side_set 1"
                ".wrap_target",
                "out x, 32      side 0b1",
                "wait 1 irq 6   side 0b1",
                "delay:",
                "jmp x-- delay  side 0b0",
                "irq 7          side 0b1",
                ".wrap",
            );
            let installed = pio_block.install(&program_data.program).unwrap();
            let (mut sm, _, tx) = PIOBuilder::from_program(installed)
                .side_set_pin_base(pins.oe.id().num)
                .clock_divisor_fixed_point(1, 1)
                .autopull(true)
                .buffers(Buffers::OnlyTx)
                .build(oe_sm);
            sm.set_pindirs([(pins.oe.id().num, PinDir::Output)]);
            (sm, tx)
        };

        // Setup DMA
        let (fb_ch, fb_loop_ch, oe_ch, oe_loop_ch) = dma_chs;

        // TODO: move this to a better place
        buffer.fbptr[0] = buffer.fb0.as_ptr() as u32;
        buffer.delaysptr[0] = buffer.delays.as_ptr() as u32;

        // Framebuffer channel
        fb_ch.regs().ch_al1_ctrl.write(|w| unsafe {
            w
                // Increase the read addr as we progress through the buffer
                .incr_read()
                .bit(true)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // Setup PIO FIFO as data request trigger
                .treq_sel()
                .bits(data_sm_tx.dreq_value())
                // Turn off interrupts
                .irq_quiet()
                .bit(!benchmark)
                // Chain to the channel selecting the framebuffers
                .chain_to()
                .bits(CH1::id())
                // Enable the channel
                .en()
                .bit(true)
        });
        fb_ch
            .regs()
            .ch_read_addr
            .write(|w| unsafe { w.bits(buffer.fbptr[0]) });
        fb_ch
            .regs()
            .ch_trans_count
            .write(|w| unsafe { w.bits((fb_bytes(W, H, B) / 4) as u32) });
        fb_ch
            .regs()
            .ch_write_addr
            .write(|w| unsafe { w.bits(data_sm_tx.fifo_address() as u32) });

        // Framebuffer loop channel
        fb_loop_ch.regs().ch_al1_ctrl.write(|w| unsafe {
            w
                // Do not increase the read addr. We always want to read a single value
                .incr_read()
                .bit(false)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // No pacing
                .treq_sel()
                .permanent()
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                // Chain it back to the channel sending framebuffer data
                .chain_to()
                .bits(CH0::id())
                // Start up the DMA channel
                .en()
                .bit(true)
        });
        fb_loop_ch
            .regs()
            .ch_read_addr
            .write(|w| unsafe { w.bits(buffer.fbptr.as_ptr() as u32) });
        fb_loop_ch
            .regs()
            .ch_trans_count
            .write(|w| unsafe { w.bits(1) });
        fb_loop_ch
            .regs()
            .ch_al2_write_addr_trig
            .write(|w| unsafe { w.bits(fb_ch.regs().ch_read_addr.as_ptr() as u32) });

        // Output enable channel
        oe_ch.regs().ch_al1_ctrl.write(|w| unsafe {
            w
                // Increase the read addr as we progress through the buffer
                .incr_read()
                .bit(true)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // Setup PIO FIFO as data request trigger
                .treq_sel()
                .bits(oe_sm_tx.dreq_value())
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                // Chain to the channel selecting the framebuffers
                .chain_to()
                .bits(CH3::id())
                // Enable the channel
                .en()
                .bit(true)
        });
        oe_ch
            .regs()
            .ch_read_addr
            .write(|w| unsafe { w.bits(buffer.delays.as_ptr() as u32) });
        oe_ch
            .regs()
            .ch_trans_count
            .write(|w| unsafe { w.bits(buffer.delays.len().try_into().unwrap()) });
        oe_ch
            .regs()
            .ch_write_addr
            .write(|w| unsafe { w.bits(oe_sm_tx.fifo_address() as u32) });

        // Output enable loop channel
        oe_loop_ch.regs().ch_al1_ctrl.write(|w| unsafe {
            w
                // Do not increase the read addr. We always want to read a single value
                .incr_read()
                .bit(false)
                // Do not increase the write addr because we always want to write to PIO FIFO
                .incr_write()
                .bit(false)
                // Read 32 bits at a time
                .data_size()
                .size_word()
                // No pacing
                .treq_sel()
                .permanent()
                // Turn off interrupts
                .irq_quiet()
                .bit(true)
                // Chain it back to the channel sending framebuffer data
                .chain_to()
                .bits(CH2::id())
                // Start up the DMA channel
                .en()
                .bit(true)
        });
        oe_loop_ch
            .regs()
            .ch_read_addr
            .write(|w| unsafe { w.bits(buffer.delaysptr.as_ptr() as u32) });
        oe_loop_ch
            .regs()
            .ch_trans_count
            .write(|w| unsafe { w.bits(buffer.delaysptr.len().try_into().unwrap()) });
        oe_loop_ch
            .regs()
            .ch_al2_write_addr_trig
            .write(|w| unsafe { w.bits(oe_ch.regs().ch_read_addr.as_ptr() as u32) });

        data_sm.start();
        row_sm.start();
        oe_sm.start();

        Display {
            mem: buffer,
            fb_loop_ch,
            benchmark,
            brightness: 255,
            lut,
        }
    }

    fn fb_loop_busy(&self) -> bool {
        self.fb_loop_ch
            .regs()
            .ch_ctrl_trig
            .read()
            .busy()
            .bit_is_set()
    }

    /// Flips the display buffers
    ///
    /// Has to be called once you have drawn something onto the currently inactive buffer.
    pub fn commit(&mut self) {
        if self.mem.fbptr[0] == (self.mem.fb0.as_ptr() as u32) {
            self.mem.fbptr[0] = self.mem.fb1.as_ptr() as u32;
            while !self.benchmark && !self.fb_loop_busy() {}
            self.mem.fb0[0..].fill(0);
        } else {
            self.mem.fbptr[0] = self.mem.fb0.as_ptr() as u32;
            while !self.benchmark && !self.fb_loop_busy() {}
            self.mem.fb1[0..].fill(0);
        }
    }

    /// Paints the given pixel coordinates with the given color
    ///
    /// Note that the coordinates are 0-indexed.
    pub fn set_pixel(&mut self, x: usize, y: usize, color: C) {
        // invert the screen
        let x = W - 1 - x;
        let y = H - 1 - y;
        // Half of the screen
        let h = y > (H / 2) - 1;
        let shift = if h { 3 } else { 0 };
        let (c_r, c_g, c_b) = self.lut.lookup(color);
        let c_r: u16 = ((c_r as f32) * (self.brightness as f32 / 255f32)) as u16;
        let c_g: u16 = ((c_g as f32) * (self.brightness as f32 / 255f32)) as u16;
        let c_b: u16 = ((c_b as f32) * (self.brightness as f32 / 255f32)) as u16;
        for b in 0..B {
            // Extract the n-th bit of each component of the color and pack them
            let cr = c_r >> b & 0b1;
            let cg = c_g >> b & 0b1;
            let cb = c_b >> b & 0b1;
            let c = cb << 2 | cg << 1 | cr;
            let idx = b * W + x + ((y % (H / 2)) * W * B);
            if self.mem.fbptr[0] == (self.mem.fb0.as_ptr() as u32) {
                self.mem.fb1[idx] &= !(0b111 << shift);
                self.mem.fb1[idx] |= (c << shift) as u8;
            } else {
                self.mem.fb0[idx] &= !(0b111 << shift);
                self.mem.fb0[idx] |= (c << shift) as u8;
            }
        }
    }

    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness
    }
}

impl<'a, CH1, const W: usize, const H: usize, const B: usize, C> OriginDimensions
    for Display<'a, CH1, W, H, B, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: ChannelIndex,
    C: RgbColor,
{
    fn size(&self) -> Size {
        Size::new(W.try_into().unwrap(), H.try_into().unwrap())
    }
}

impl<'a, CH1, const W: usize, const H: usize, const B: usize, C> DrawTarget
    for Display<'a, CH1, W, H, B, C>
where
    [(); fb_bytes(W, H, B)]: Sized,
    CH1: ChannelIndex,
    C: RgbColor,
{
    type Color = C;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x < W.try_into().unwrap() && coord.y < H.try_into().unwrap() {
                self.set_pixel(coord.x as usize, coord.y as usize, color);
            }
        }

        Ok(())
    }
}
