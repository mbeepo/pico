use embassy_rp::{peripherals::PIO0, pio::{self, Common, PioPin, ShiftConfig, ShiftDirection, StateMachine}};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;

use crate::serial::UninitSerialPair;

pub fn setup_7sd_task_sm0<'a, ST, SU, DT, DU>(pio: &mut Common<'a, PIO0>, sm: &mut StateMachine<'a, PIO0, 0>, segment_pins: UninitSerialPair<ST, SU>, digit_pins: UninitSerialPair<DT, DU>, segment_rclk: impl PioPin) 
        where ST: PioPin, SU: PioPin, DT: PioPin, DU: PioPin {
    let prog = pio_proc::pio_asm!(
        ".side_set 1",
        "   set x, 4 side 0",
        "   set pins, 0b010 side 0 [1]",
        "   set pins, 0b011 side 0",
        "digit:"
        "   set pins, 0b101 side 0",
        // wait 2.56ms (32 * 8 * 10us)
        "   set y, 31 side 0",
        "waitloop:",
        "   jmp y--, waitloop side 0 [7]"
        "   set pins, 0b000 side 0",
        "   set y, 7 side 0",
        "segment:",
        "   out pins, 1 side 0b0",
        "   jmp y--, segment side 0b1",
        "   jmp x--, digit side 0"
    );
    
    let mut cfg = pio::Config::default();
    let digit_out = digit_pins.init_out(pio, sm);
    let segment_out = segment_pins.init_out(pio, sm);
    let segment_rclk = pio.make_pio_pin(segment_rclk);
    sm.set_pin_dirs(pio::Direction::Out, &[&segment_rclk]);
    cfg.use_program(&pio.load_program(&prog.program), &[&segment_rclk, &digit_out.data, &digit_out.clock]);
    // cfg.use_program(&pio.load_program(&prog.program), &[&segment_out.clock]);
    cfg.clock_divider = (U56F8!(125_000_000) / U56F8!(10_000)).to_fixed();
    cfg.set_out_pins(&[&segment_out.data]);
    cfg.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: 32,
        direction: ShiftDirection::Right,
    };

    sm.set_config(&cfg);
}