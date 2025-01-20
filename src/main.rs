#![no_std]
#![no_main]

use core::sync::atomic::{compiler_fence, Ordering};

use defmt::unwrap;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{bind_interrupts, dma::Channel, gpio, multicore::{spawn_core1, Stack}, peripherals::PIO0, pio::{self, Common, Instance, Pin, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine}};
use embassy_time::Timer;
use fixed_macro::types::U56F8;
use gpio::{Level, Output};
use rp_pac::dma::vals::{DataSize, TreqSel};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use fixed::traits::ToFixed;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

const SM0_TXF0: *mut u32 = 0x50200010 as *mut u32;

static HAHA: u32 = 1;
static mut OUTPUT: u32 = 0b10101010_10101010_10101010_10101010;

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_16, Level::Low);

    let mut pio = Pio::new(p.PIO0, Irqs);
    let segment_pins = UninitSerialPair { data: p.PIN_15, clock: p.PIN_14 };
    let digit_pins = UninitSerialPair { data: p.PIN_13, clock: p.PIN_12 };
    setup_7sd_task_sm0(&mut pio.common, &mut pio.sm0, segment_pins, digit_pins);

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

    loop {
        led.set_low();
        Timer::after_micros(975).await;
        led.set_high();
        Timer::after_micros(25).await;
    }
}

#[embassy_executor::task]
async fn core1_silly() {
    Timer::after_millis(2000).await;
    unsafe { OUTPUT = u32::MAX; }

    loop {}
}

fn init_ch<C: Channel>(
    ch: &C,
    from: *const u32,
    to: *mut u32,
    datasize: DataSize,
    chain_to: u8,
    quiet: bool,
) {
    let p = ch.regs();

    p.read_addr().write_value(from as u32);
    p.write_addr().write_value(to as u32);
    p.trans_count().write_value(1);

    compiler_fence(Ordering::SeqCst);

    p.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel::PERMANENT);
        w.set_data_size(datasize);
        w.set_incr_read(false);
        w.set_incr_write(false);
        w.set_chain_to(chain_to);
        w.set_irq_quiet(quiet);
        w.set_en(true);
    });

    compiler_fence(Ordering::SeqCst);
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
    fn init_in<'a, B: Instance>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, 0>) -> SerialInput<'a, B> {
        let pins = self.init_helper(pio, sm, pio::Direction::In);
        SerialInput { data: pins.0, clock: pins.1 }
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
        ".wrap_target",
        "set x, 5 side 0b110", // decrementing at start of loop so we need to start 1 higher
        "nop [1]", // wait a couple bonus cycles just to be sure it displays
        "nop [1]",
        "nop [1]",
        "digit:",
        "jmp x--, digit side 0b100",
        "set y, 7", // shift all 8 bits out and leave pin A of the segment SR unused (this is a surprise tool that will help us later)
        "segment:",
            "out pins, 1 side 0b001"
            "jmp y--, segment side 0b000",
        ".wrap",
    );

    /*
        x = 5, push 1 to digit
        wait 6 cycles
        x-- (4)
        y = 8
        push 0 to segment
        y-- (7)
        push 1 to segment
        y-- (6)
        push 0 to segment
        y-- (5)
        push 1 to segment
        y-- (4)
        push 0 to segment
        y-- (3)
        push 0 to segment
        y-- (3)
        push 0 to segment
        y-- (3)
        push 0 to segment
        y-- (3)
        push 0 to segment
        y-- (3)
    */
    
    let mut cfg = pio::Config::default();
    let digit_out = digit_pins.init_out(pio, sm);
    let segment_out = segment_pins.init_out(pio, sm);
    cfg.use_program(&pio.load_program(&prog.program), &[&digit_out.clock, &digit_out.data, &segment_out.clock]);
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
    cfg.set_out_pins(&[&segment_out.data]);
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Right,
    };

    sm.set_config(&cfg);
}