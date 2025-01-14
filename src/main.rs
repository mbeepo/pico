#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, gpio, interrupt, peripherals::PIO0, pio::{self, Common, Pio, PioPin, StateMachine}};
use embassy_time::Timer;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use fixed_macro::types::U56F8;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use fixed::traits::ToFixed;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_16, Level::Low);
    let mut counter: u8 = 0;
    let mut pio = Pio::new(p.PIO0, Irqs);

    loop {
        info!("led on!");
        led.set_high();
        Timer::after_millis(counter as u64 * 4).await;

        info!("led off!");
        led.set_low();
        Timer::after_millis(counter as u64 * 4).await;

        counter = counter.wrapping_add(1);
    }
}

fn setup_pio_task_sm0<'a>(pio: &mut Common<'a, PIO0>, sm: &mut StateMachine<'a, PIO0, 0>,
        segment_pins: (impl PioPin, impl PioPin, impl PioPin, impl PioPin, impl PioPin, impl PioPin, impl PioPin),
        digit_pins: (impl PioPin, impl PioPin, impl PioPin, impl PioPin)) {
    /*
        Pins:
            Out to shift register
            Shift register clock
            Out to shift register 2
            Shift register 2 clock
    */
    let prog = pio_proc::pio_asm!(
        ".wrap_target",
        "out pins, 11 [24]",
        "nop [24]",
        "nop [24]",
        "nop [24]",
        "nop [24]",
        ".wrap",
    );

    let out_pins = (
        pio.make_pio_pin(digit_pins.0), pio.make_pio_pin(digit_pins.1),
        pio.make_pio_pin(digit_pins.2), pio.make_pio_pin(digit_pins.3),
        pio.make_pio_pin(segment_pins.0), pio.make_pio_pin(segment_pins.1),
        pio.make_pio_pin(segment_pins.2), pio.make_pio_pin(segment_pins.3),
        pio.make_pio_pin(segment_pins.4), pio.make_pio_pin(segment_pins.5),
        pio.make_pio_pin(segment_pins.6),
    );

    sm.set_pin_dirs(pio::Direction::Out, &[
        &out_pins.0, &out_pins.1, &out_pins.2, &out_pins.3, 
        &out_pins.4, &out_pins.5, &out_pins.6, &out_pins.7, 
        &out_pins.8, &out_pins.9, &out_pins.10, 
    ]);

    let mut cfg = pio::Config::default();
    
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
    cfg.use_program(&pio.load_program(&prog.program), &[]);
    cfg.set_out_pins(&[
        &out_pins.0, &out_pins.1, &out_pins.2, &out_pins.3, 
        &out_pins.4, &out_pins.5, &out_pins.6, &out_pins.7, 
        &out_pins.8, &out_pins.9, &out_pins.10, 
    ]);
    cfg.shift_out.auto_fill = true;
    sm.set_config(&cfg);
}

#[embassy_executor::task]
async fn pio_task_sm0(mut sm: StateMachine<'static, PIO0, 0>) {
    sm.set_enable(true);
}