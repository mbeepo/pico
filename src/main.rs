#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, dma, gpio, interrupt, peripherals::PIO0, pio::{self, Common, Config, Instance, Pin, Pio, PioPin, StateMachine}, Peripheral};
use embassy_time::Timer;
use fixed_macro::types::U56F8;
use gpio::{Level, Output};
use {defmt_rtt as _, panic_probe as _};
use fixed::traits::ToFixed;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_16, Level::Low);
    let mut counter: u8 = 0;

    let mut pio = Pio::new(p.PIO0, Irqs);
    let segment_pins = UninitSerialPair { data: p.PIN_15, clock: p.PIN_14 };
    let digit_pins = UninitSerialPair { data: p.PIN_13, clock: p.PIN_12 };
    setup_7sd_task_sm0(&mut pio.common, &mut pio.sm0, segment_pins, digit_pins);
    pio.sm0.tx().dma_push(p.DMA_CH0.into_ref(), &[0b10101010101010101010101010101010u32]).await;
    spawner.spawn(pio_task_sm0(pio.sm0)).unwrap();


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

struct UninitSerialPair<T, U>
        where T: PioPin, U: PioPin {
    data: T,
    clock: U,
}

impl<T, U> UninitSerialPair<T, U>
        where T: PioPin, U: PioPin {
    /// Initializes the pair as output pins for the passed state machine
    fn init_out<'a, B: Instance>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, 0>) -> SerialOutput<'a, B> {
        let pins = self.init_helper(pio, sm, pio::Direction::Out);
        SerialOutput { data: pins.0, clock: pins.1 }
    }
    
    /// Initializes the pair as output pins for the passed state machine
    fn init_in<'a, B: Instance>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, 0>) -> SerialOutput<'a, B> {
        let pins = self.init_helper(pio, sm, pio::Direction::In);
        SerialOutput { data: pins.0, clock: pins.1 }
    }

    fn init_helper<'a, B: Instance, const SM: usize>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, SM>, dir: pio::Direction) -> (Pin<'a, B>, Pin<'a, B>) {
        let pins = (pio.make_pio_pin(self.data), pio.make_pio_pin(self.clock));
        sm.set_pin_dirs(dir, &[&pins.0, &pins.1]);
        pins
    }
}

struct SerialOutput<'a, B: Instance> {
    data: Pin<'a, B>,
    clock: Pin<'a, B>,
}

struct SerialInput<'a, B: Instance> {
    data: Pin<'a, B>,
    clock: Pin<'a, B>,
}

fn setup_7sd_task_sm0<'a, ST, SU, DT, DU>(pio: &mut Common<'a, PIO0>, sm: &mut StateMachine<'a, PIO0, 0>, segment_pins: UninitSerialPair<ST, SU>, digit_pins: UninitSerialPair<DT, DU>) 
        where ST: PioPin, SU: PioPin, DT: PioPin, DU: PioPin {
    let prog = pio_proc::pio_asm!(
        ".side_set 3 opt",
        "start:",
        "set x, 5 side 0b011", // decrementing at start of loop so we need to start 1 higher
        "digit:",
            "out null, 1",
            "jmp x--, start side 0b001",
            "nop [1]", // wait a couple bonus cycles just to be sure it displays
            "nop [1]",
            "nop [1]",
            "set y, 8", // shift all 8 bits out and leave pin A of the segment SR unused (this is a surprise tool that will help us later)
            ".wrap_target",
                "out pins, 1 side 0b100"
                "set pins, 0b000",
                "jmp y--, digit",
            ".wrap",
    );
    
    let segment_out = segment_pins.init_out(pio, sm);
    let digit_out = digit_pins.init_out(pio, sm);

    let mut cfg = pio::Config::default();
    
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
    cfg.use_program(&pio.load_program(&prog.program), &[]);
    cfg.set_out_pins(&[&segment_out.data]);
    cfg.set_set_pins(&[&segment_out.clock, &digit_out.data, &digit_out.clock]);
    cfg.shift_out.auto_fill = true;
    sm.set_config(&cfg);
}

#[embassy_executor::task]
async fn pio_task_sm0(mut sm: StateMachine<'static, PIO0, 0>) {
    sm.set_enable(true);
}