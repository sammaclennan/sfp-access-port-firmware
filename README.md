# SFP-Access-Port
Software supporting the SFP Access Port hardware

The SFP Access Port project is an open source effort to create the hardware and associated software capable of reading and writing to the embedded EEPROM found on SFP and SFP+ transceiver modules, as well as other transceivers compliant with the relevant standards.

## Why?
All SFP modules contain an onboard EEPROM - this EEPROM contains information about the modules itself, including it's capabilities, compatibilities and vendor origin. The ability to extract this information from transceivers can be beneficial for a number of reason.

SFP modules are frequently coded with specific hardware vendors with which it should be considered compatible - despite the hardware being identical to other modules. Conversely, some network appliance vendors will lock their devices to only work correctly with specific modules.

This project provides the tools required to read and write this information from compliant modules under certain conditions.

## Building 

Prior to building the project, verify you have the latest stable Rust installed, as well as the correct targets

```sh
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
```
You will additionally want the following tools, for flashing and debugging

```sh
cargo install elf2uf2-rs --locked
cargo install probe-run
```

With these dependencies installed, it the program should build with `cargo build`.

## Programming

The easiest way to get your code on the board is using the RP2040's UF2 over USB capabilities. 

Rust generates standard ARM ELF files, which can be flashed directly to the RP2040 using a external programmer, however the RP2040 ROM bootloader includes a Mass Storage Device that can be used to program the chip without anything more than a USB cable. This method requires UF2 format images, which can be generated from ARM ELF images using the elf2uf2-rs package installed above.

This cargo environment is configured to automatically run the elf2uf2 package when executing `cargo run`. If a RP2040 Mass Storage Device is present, the application will also automatically be copied to the device.
## Project Goals

Coming Soon!

