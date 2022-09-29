#![no_std]
#![no_main]


use core::sync::atomic::{AtomicBool, Ordering};

// Define behavior on panic
use panic_halt as _;

use rp2040_hal as hal;
use hal::{pac, I2C};
use pac::interrupt;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use fugit::RateExtU32;
use embedded_hal::{
    digital::v2::OutputPin, 
    prelude::_embedded_hal_blocking_i2c_Read
};
use rp2040_hal::clocks::Clock;
use rp2040_boot2;   

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

//Global USB Drivers (shared with Interrupts)
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

static XCVR_PRSNT: AtomicBool = AtomicBool::new(false);
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

    // Setup USB Bus Driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS, 
        pac.USBCTRL_DPRAM, 
        clocks.usb_clock, 
        true, 
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe - interrupts have not been started
        USB_BUS = Some(usb_bus);
    }
    let bus_ref = unsafe {
        USB_BUS.as_ref().unwrap()
    };

    let mut serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    let mut usb_dev = UsbDeviceBuilder::new(
        bus_ref, UsbVidPid(0x1209, 0x0009))
        .manufacturer("MacLennan.dev")
        .product("SFP Access Port v0.1")
        .serial_number("0x0001")
        .device_class(2)
        .build();
    unsafe{
        USB_DEVICE = Some(usb_dev)
    }

    // Enable USB Interrupts
    unsafe{
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    
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
    //serial.write(b"SFP Access Port v0.1 \r\n");

    loop {

            let mut readbuf: [u8; 1] = [0; 1];
            let res = i2c.read(0x50,&mut readbuf);
            if let Ok(d) = res {
                XCVR_PRSNT.store(true, Ordering::Relaxed)
            }
        
        // Check for new data
        // if usb_dev.poll(&mut [&mut serial]) {
        //     if (device_found){
        //         serial.write(b"Found SFP Device!\r\n");
        //         device_found = false;
        //     }else {
        //         serial.write(b"No Device Found YET!\r\n");
        //     }
            
        // }
        //delay.delay_ms(2000);
    }

}

// Called whenever a USB_IRQ Interrupt Request is generated
// All USB work is done under this interrupt

#[interrupt]
unsafe fn  USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};

    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    if usb_dev.poll(&mut [serial]){
        let  mut buf = [0u8; 64];
        match serial.read(&mut buf){
            Err(_e) => {
                // Ignore
            }
            Ok(0) => {
                //No data - Ignore
            }
            Ok(count) => {
                if XCVR_PRSNT.load(Ordering::Relaxed) {
                    serial.write(b"SFP Module Detected!\r\n");
                    XCVR_PRSNT.store(false, Ordering::Relaxed)
                } else {
                    serial.write(b"No SFP Module Detected\r\n");
                }
            }
        }
    }

}

