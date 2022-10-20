#![no_std]
#![no_main]


use core::{sync::atomic::{AtomicBool, Ordering}, cell::RefCell, ops::DerefMut};

use cortex_m::{asm::wfi};
use critical_section::{Mutex};
// Define behavior on panic
use panic_halt as _;

use rp2040_hal as hal;
use hal::{pac::{self, I2C0, interrupt}, I2C, gpio::{self, bank0::Gpio28,bank0::Gpio29, Function}};
use gpio::{Pin};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

use fugit::RateExtU32;
use embedded_hal::{
    prelude::_embedded_hal_blocking_i2c_Read, digital::v2::OutputPin
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

//Global Peripheral Access
type I2C_PINS = (Pin<Gpio28, Function<gpio::I2C>>, Pin<Gpio29, Function<gpio::I2C>>);
static I2C_BUS: Mutex<RefCell<Option<I2C<I2C0,I2C_PINS>>>> = Mutex::new(RefCell::new(None));


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
        bus_ref, UsbVidPid(0x1209, 0xFA00))
        .manufacturer("Maclennan.dev")
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
    // Move our I2C struct into our RefCell - can't do this statically, so it needs to be done at runtime
    critical_section::with(|cs| I2C_BUS.borrow(cs).replace(Some(i2c)));


    let mut aux_led = pins.gpio20.into_push_pull_output();
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    //serial.write(b"SFP Access Port v0.1 \r\n");

    loop {
        aux_led.set_high().unwrap();
        delay.delay_ms(1000);
        aux_led.set_low().unwrap();
        delay.delay_ms(1000);
    }

}

// Called whenever a USB_IRQ Interrupt Request is generated
// All USB work is done under this interrupt

#[interrupt]
unsafe fn  USBCTRL_IRQ() {

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
                critical_section::with(|cs| {
                    let mut readbuf: [u8; 127] = [0;127]; 

                    // We need a mutable reference to the I2C bus, so we borrow and de-ref
                    if let Some(ref mut i2c_ref) = I2C_BUS.borrow(cs).borrow_mut().deref_mut(){

                        // Technically this is blocking, so we might not want this in an interrupt - but it should be okay for now??
                        let res = i2c_ref.read(0x50, &mut readbuf);
                        if let Ok(d) = res {
                           serial_write(serial, "XCVR Detected!", false);
                           serial_write(serial, unsafe {core::str::from_utf8_unchecked(&readbuf)}, false); // When I get a global allocator working we can safe-ify this
                           serial_write(serial, "\r\n", false);
                           readbuf = [0;127];
                        } else {
                            serial_write(serial, "No XCVR Detected... \r\n", false);
                        }
                    };
                    
                })
                
            }
        }
    }

}

// Quick helper function to coerce data into a serial interface
fn serial_write(serial: &mut SerialPort<'static, hal::usb::UsbBus>, buf: &str, block:bool){
    let buf_ptr = buf.as_bytes();

    // Test the size of the data we want to send - then shrink our buffer.
    let mut index = 0;
    while index < buf_ptr.len() && buf_ptr[index] != 0 {
        index += 1;
    }
    let mut buf_ptr = &buf_ptr[0..index];

    while !buf_ptr.is_empty() {
        match serial.write(buf_ptr) {
            Ok(len) => buf_ptr = &buf_ptr[len..],
            Err(UsbError::WouldBlock) => {
                //USB Write Buffer is full - we really don't want this to block if called from an interrupt!
                if !block {
                    break;
                }
            }
            Err(_) => break, //Any other errors, we can just drop the data and handle the consequences later
        }
    }
    let _ = serial.flush();
}

