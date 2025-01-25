#![no_std]
#![no_main]

mod pio_programs;
mod dma;
mod serial;

use dma::init_ch;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, dma::Channel, multicore::Stack, peripherals::PIO0, pio::{self, Pio}};
use embassy_time::Timer;
use pio_programs::segmented_display::setup_7sd_task_sm0;
use rp_pac::dma::vals::DataSize;
use serial::UninitSerialPair;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

const SM0_TXF0: *mut u32 = 0x50200010 as *mut u32;

static HAHA: u32 = 1;
static mut OUTPUT: u32 = 0b10000000_10000000_10000000_10000000;
// static mut OUTPUT: u32 = u32::MAX;

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut pio = Pio::new(p.PIO0, Irqs);
    let segment_pins = UninitSerialPair { data: p.PIN_11, clock: p.PIN_12 };
    let segment_rclk = p.PIN_13;
    let digit_pins = UninitSerialPair { data: p.PIN_14, clock: p.PIN_15 };

    setup_7sd_task_sm0(&mut pio.common, &mut pio.sm0, segment_pins, digit_pins, segment_rclk);

    pio.sm0.set_enable(true);

    init_ch(&p.DMA_CH0, &raw const HAHA, p.DMA_CH1.regs().al1_trans_count_trig().as_ptr(), DataSize::SIZE_BYTE, p.DMA_CH0.number(), true);
    init_ch(&p.DMA_CH1, &raw const OUTPUT, SM0_TXF0, DataSize::SIZE_WORD, p.DMA_CH0.number(), true);

    // spawn_core1(
    //     p.CORE1,
    //     unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
    //     move || {
    //         let executor1 = EXECUTOR1.init(Executor::new());
    //         executor1.run(|spawner| unwrap!(spawner.spawn(core1_silly())));
    //     }
    // );

    loop {}
}

#[embassy_executor::task]
async fn core1_silly() {
    Timer::after_millis(2000).await;
    unsafe { OUTPUT = u32::MAX; }

    loop {}
}