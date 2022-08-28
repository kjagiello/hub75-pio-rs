# hub75-pio-rs

An **experimental** HUB75 driver for
[RP2040](https://www.raspberrypi.com/products/rp2040/). Utilizes Programmable
I/O (PIO) units in combination with DMA to achieve high refresh rates, true
color depth and zero CPU overhead without sacrificing quality.

https://user-images.githubusercontent.com/74944/187094663-2f52e020-ccb2-4103-b69b-af8ee2185dd0.mp4

## Features

- Supports LED matrices up to 64x32 pixels with 1:16 scanline
- High refresh rate (approx. 2100 Hz with 24 bit color depth on a 64x32
  display)
- Does not utilize CPU for clocking out data to the display – all the work is
  done by the PIOs and the DMA controller
- Uses binary color modulation
- Double buffered
- Implements [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) traits

## Requirements

The current implementation assumes that the following groups of data outputs
are assigned to consecutive pins on the RP2040:

- R1, G1, B1, R2, G2, B2
- ADDRA, ADDRB, ADDRC, ADDRD
