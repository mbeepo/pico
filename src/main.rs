#![no_std]
#![no_main]

mod pio_programs;
mod display_driver;
mod dma;
mod serial;

use display_driver::{FourDigit7Sd, FourDigit7SdDriver, FourDigit7SdRaw};
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, multicore::Stack, peripherals::PIO0, pio::{self, Pio}};
use embassy_time::Timer;
use serial::UninitSerialPair;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut pio = Pio::new(p.PIO0, Irqs);
    let segment_pins = UninitSerialPair { data: p.PIN_11, clock: p.PIN_12 };
    let segment_rclk = p.PIN_13;
    let digit_pins = UninitSerialPair { data: p.PIN_14, clock: p.PIN_15 };

    let mut display = FourDigit7SdDriver::init(p.DMA_CH0, p.DMA_CH1, &mut pio, segment_pins, digit_pins, segment_rclk);
    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(|spawner| unwrap!(spawner.spawn(core1_silly())));
    //     }
    // );
    let mut counter = 1;

    loop {
        display.set_7sd(FourDigit7Sd::try_from_dec(counter).unwrap());
        
        counter += 1;
        if counter > 9999 {
            counter = 1;
        }

        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn core1_silly() {
    Timer::after_millis(2000).await;
    // set_7sd(u32::MAX.into());

    loop {}
}