//! Displays a procedurally generated flame
//!
//! Based on https://indigoengine.io/docs/guides/howto-fire-shader
#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::pixelcolor::Rgb888;
use panic_probe as _;

use bsp::hal::pio::PIOExt;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use hub75_pio;
use hub75_pio::dma::DMAExt;
use hub75_pio::lut::GammaLut;

use rp_pico as bsp;

mod flame;

static mut DISPLAY_BUFFER: hub75_pio::DisplayMemory<64, 32, 12> = hub75_pio::DisplayMemory::new();

#[entry]
fn main() -> ! {
    info!("Program start");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Split PIO0 SM
    let (mut pio, sm0, sm1, sm2, _) = pac.PIO0.split(&mut pac.RESETS);

    // Reset DMA
    let resets = pac.RESETS;
    resets.reset.modify(|_, w| w.dma().set_bit());
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while resets.reset_done.read().dma().bit_is_clear() {}

    // Split DMA
    let dma = pac.DMA.split();

    let lut = {
        let lut: GammaLut<12, _, _> = GammaLut::new();
        lut.init((1.0, 1.0, 1.0))
    };
    let mut display = unsafe {
        hub75_pio::Display::new(
            &mut DISPLAY_BUFFER,
            hub75_pio::DisplayPins {
                r1: pins.gpio0.into(),
                g1: pins.gpio1.into(),
                b1: pins.gpio2.into(),
                r2: pins.gpio3.into(),
                g2: pins.gpio4.into(),
                b2: pins.gpio5.into(),
                addra: pins.gpio6.into(),
                addrb: pins.gpio7.into(),
                addrc: pins.gpio8.into(),
                addrd: pins.gpio9.into(),
                clk: pins.gpio11.into(),
                lat: pins.gpio12.into(),
                oe: pins.gpio13.into(),
            },
            &mut pio,
            (sm0, sm1, sm2),
            (dma.ch0, dma.ch1, dma.ch2, dma.ch3),
            false,
            &lut,
        )
    };

    let mut t = 0;
    loop {
        let canvas = flame::tick(t);
        for (y, row) in canvas.row_iter().enumerate() {
            for (x, _) in row.column_iter().enumerate() {
                let a = canvas[(x, y)];
                let r: u8 = *a.get(0).unwrap();
                let g: u8 = *a.get(1).unwrap();
                let b: u8 = *a.get(2).unwrap();

                display.set_pixel(x + 14, y, Rgb888::new(r, g, b));
            }
        }
        display.commit();
        delay.delay_ms(10);
        t = (t + 1) % 2000;
    }
}
