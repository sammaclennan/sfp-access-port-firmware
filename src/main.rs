#![no_std]
#![no_main]



use panic_halt as _;
use rp2040_hal as hal;
use hal::{pac, I2C, i2c};
use usb_device::{class_prelude::*, prelude::*, device};
use usbd_serial::SerialPort; // Unmaintained

use fugit::RateExtU32;
use embedded_hal::{digital::v2::OutputPin, prelude::_embedded_hal_blocking_i2c_Read};
use rp2040_hal::clocks::Clock;
use rp2040_boot2;   

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;


#[rp2040_hal::entry]
fn main() -> ! {

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ, 
        pac.XOSC, 
        pac.CLOCKS, 
        pac.PLL_SYS, 
        pac.PLL_USB, 
        &mut pac.RESETS, 
        &mut watchdog).ok().unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS, 
        pac.USBCTRL_DPRAM, 
        clocks.usb_clock, 
        true, 
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(
        &usb_bus, UsbVidPid(0x1209, 0x0009))
        .manufacturer("maclennan")
        .product("SFP-Access-Port-v0.1")
        .serial_number("0x0001")
        .device_class(2)
        .build();
    
    let mut i2c = I2C::i2c0(
        pac.I2C0, 
        pins.gpio28.into_mode(), 
        pins.gpio29.into_mode(), 
        100.kHz(), 
        &mut pac.RESETS, 
        12_000_000.Hz()
    );

    let mut aux_led = pins.gpio20.into_push_pull_output();
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut device_found = false;
    //serial.write(b"SFP Access Port v0.1 \r\n");

    loop {

            let mut readbuf: [u8; 1] = [0; 1];
            let res = i2c.read(0x50,&mut readbuf);
            if let Ok(d) = res {
                device_found = true;
            }
        
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            if (device_found){
                serial.write(b"Found SFP Device!\r\n");
            }else {
                serial.write(b"No Device Found YET!\r\n");
            }
        }
    }

}

