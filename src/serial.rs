use embassy_rp::pio::{self, Common, Instance, Pin, PioPin, StateMachine};

pub struct UninitSerialPair<T, U>
        where T: PioPin, U: PioPin {
    pub data: T,
    pub clock: U,
}

impl<T, U> UninitSerialPair<T, U>
        where T: PioPin, U: PioPin {
    /// Initializes the pair as output pins for the passed state machine
    pub fn init_out<'a, B: Instance>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, 0>) -> SerialOutput<'a, B> {
        let pins = self.init_helper(pio, sm, pio::Direction::Out);
        SerialOutput { data: pins.0, clock: pins.1 }
    }
    
    /// Initializes the pair as output pins for the passed state machine
    pub fn init_in<'a, B: Instance>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, 0>) -> SerialInput<'a, B> {
        let pins = self.init_helper(pio, sm, pio::Direction::In);
        SerialInput { data: pins.0, clock: pins.1 }
    }

    pub fn init_helper<'a, B: Instance, const SM: usize>(self, pio: &mut Common<'a, B>, sm: &mut StateMachine<'a, B, SM>, dir: pio::Direction) -> (Pin<'a, B>, Pin<'a, B>) {
        let pins = (pio.make_pio_pin(self.data), pio.make_pio_pin(self.clock));
        sm.set_pin_dirs(dir, &[&pins.0, &pins.1]);
        pins
    }
}

pub struct SerialOutput<'a, B: Instance> {
    pub data: Pin<'a, B>,
    pub clock: Pin<'a, B>,
}

pub struct SerialInput<'a, B: Instance> {
    pub data: Pin<'a, B>,
    pub clock: Pin<'a, B>,
}