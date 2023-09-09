//! Displays a procedurally generated flame
//!
//! Based on https://indigoengine.io/docs/guides/howto-fire-shader
#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use bsp::entry;
use core::ptr;
use defmt::*;
use defmt_rtt as _;
use embedded_graphics::{pixelcolor::Rgb888, prelude::*};
use panic_probe as _;

use bsp::hal::pio::PIOExt;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};
use hub75_pio;
use hub75_pio::dma::DMAExt;
use hub75_pio::lut::GammaLut;

use core::cell::RefCell;
use core::str;
use critical_section::Mutex;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    text::Text,
};
use rp_pico as bsp;

static mut DISPLAY_BUFFER: hub75_pio::DisplayMemory<64, 32, 12> = hub75_pio::DisplayMemory::new();
static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0u32));

fn hz_to_str(mut n: u32, buf: &mut [u8]) -> &[u8] {
    if n == 0 {
        return b"0";
    }
    let mut i = 3;
    while n > 0 {
        buf[i] = (n % 10) as u8 + b'0';
        n /= 10;
        i += 1;
    }
    buf[0] = 'z' as u8;
    buf[1] = 'H' as u8;
    buf[2] = ' ' as u8;
    let slice = &mut buf[..i];
    slice.reverse();
    &*slice
}

#[entry]
fn main() -> ! {
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
    let dma = &pac.DMA;
    dma.inte0.write(|w| unsafe { w.bits(1 << 0) });

    let dma = pac.DMA.split();

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);
    }

    let lut = {
        let lut: GammaLut<12, _, _> = GammaLut::new();
        lut.init((2.1, 2.1, 2.1))
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
            true,
            &lut,
        )
    };

    let style = MonoTextStyle::new(&FONT_6X10, Rgb888::WHITE);
    let mut last_value = 0;
    let mut hz = 0;

    loop {
        let mut buf = [0u8; 20];
        let buf = hz_to_str(hz, &mut buf);
        let buf = str::from_utf8(&buf).unwrap();
        Text::new(buf, Point::new(12, 19), style)
            .draw(&mut display)
            .unwrap();
        display.commit();
        delay.delay_ms(1000);

        let counter = critical_section::with(|cs| *COUNTER.borrow_ref_mut(cs));
        if counter > last_value {
            hz = counter - last_value;
        }
        last_value = counter;
    }
}

#[interrupt]
fn DMA_IRQ_0() {
    critical_section::with(|cs| {
        COUNTER.replace_with(cs, |counter| (*counter + 1) % 100000000);
    });
    // Clear the DMA interrupt flag
    const INTS: *mut u32 = (0x50000000 + 0x40c) as *mut u32;
    unsafe { ptr::write_volatile(INTS, 0b1) };
}
