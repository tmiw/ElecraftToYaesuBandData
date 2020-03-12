# Introduction

This is an Arduino sketch that provides Yaesu FT-817/818 compatible voltages based on Elecraft band data transmitted
over the radio's RS-232 port. Originally developed for interfacing the Chinese-made MiniPA50 amplifier and the Arduino MKR Zero,
but can be adapted for any Arduino (with necessary configuration and/or serial port/hardware modifications) and any 
hardware that can understand FT-817/818 band data voltages.

# Usage

For the MKR Zero, use the following pins:

* Radio: pins 0 and 1 (TTL; ensure you have something like [this](https://www.sparkfun.com/products/11189) between those pins and your radio to prevent damage).
* Band data output: A0/DAC0

Set RADIO_BAUD and COMPUTER_BAUD in the sketch to preferred values (ensuring that the radio is also set to RADIO_BAUD), then 
simply plug the board into a power source (and/or your computer). The sketch will configure the radio to automatically receive
band data changes and operate on that data accordingly.

# Known Issues

TBD. Please feel free to create PRs and/or let me know of issues you find.
