use embassy_rp::{dma::Channel, peripherals::PIO0, pio::{Pio, PioPin}};
use rp_pac::dma::vals::DataSize;

use crate::{dma::init_ch, pio_programs::segmented_display::setup_7sd_task_sm0, serial::UninitSerialPair};

static HAHA: u32 = 1;
static mut OUTPUT: u32 = u32::MAX;
static mut INITIALIZED: bool = false;
static mut NEXT_OUTPUT: u32 = u32::MAX;

/// State machine 0 TX FIFO
const SM0_TXF0: *mut u32 = 0x50200010 as *mut u32;

#[derive(Clone, Copy, Debug)]
pub enum SevenSegmentDigit {
    Zero,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
    Nine,
    A, B,
    C, D,
    E, F,
    Blank,
}

impl TryFrom<u8> for SevenSegmentDigit {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x0 => Ok(Self::Zero),
            0x1 => Ok(Self::One),
            0x2 => Ok(Self::Two),
            0x3 => Ok(Self::Three),
            0x4 => Ok(Self::Four),
            0x5 => Ok(Self::Five),
            0x6 => Ok(Self::Six),
            0x7 => Ok(Self::Seven),
            0x8 => Ok(Self::Eight),
            0x9 => Ok(Self::Nine),
            0xA => Ok(Self::A),
            0xB => Ok(Self::B),
            0xC => Ok(Self::C),
            0xD => Ok(Self::D),
            0xE => Ok(Self::E),
            0xF => Ok(Self::F),
            _ => Err(()),
        }
    }
}

impl From<&SevenSegmentDigit> for u8 {
    fn from(value: &SevenSegmentDigit) -> Self {
        use SevenSegmentDigit::*;
        match value {
            Zero => 0b01111110,
            One => 0b00110000,
            Two => 0b01101101,
            Three => 0b01111001,
            Four => 0b00110011,
            Five => 0b01011011,
            Six => 0b01011111,
            Seven => 0b01110000,
            Eight => 0b01111111,
            Nine => 0b01111011,
            A => 0b01110111,
            B => 0b00011111,
            C => 0b01001111,
            D => 0b00111101,
            E => 0b01001111,
            F => 0b01000111,
            Blank => 0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FourDigit7SdRaw {
    inner: u32,
}

impl From<u32> for FourDigit7SdRaw {
    fn from(value: u32) -> Self {
        Self { inner: value }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FourDigit7Sd {
    digits: [SevenSegmentDigit; 4],
}

#[derive(Clone, Copy, Debug)]
pub enum SevenSegmentConversionError {
    TooManyDecimalDigits,
}

impl FourDigit7Sd {
    pub fn from_hex(val: u16) -> Self {
        let mut digits: [SevenSegmentDigit; 4] = [SevenSegmentDigit::Zero; 4];
        for place in 0..4 {
            let digit = ((val >> (4 * place)) & 0xF) as u8;
            digits[3 - place] = digit.try_into().unwrap_or(SevenSegmentDigit::One);
        }

        Self { digits }
    }

    pub fn try_from_dec(mut val: u16) -> Result<Self, SevenSegmentConversionError> {
        if val > 9999 { return Err(SevenSegmentConversionError::TooManyDecimalDigits) };
        let mut digits: [SevenSegmentDigit; 4] = [SevenSegmentDigit::Zero; 4];

        for place in 0..4 {
            if val == 0 {
                digits[3 - place as usize] = SevenSegmentDigit::Blank;
            } else {
                let digit = (val % 10) as u8;
                val /= 10;
                digits[3 - place as usize] = digit.try_into().unwrap();
            }
        }

        Ok(Self { digits })
    }
}

pub trait FourDigit7SdOutput {
    fn as_raw(&self) -> u32;
}

impl FourDigit7SdOutput for FourDigit7SdRaw {
    fn as_raw(&self) -> u32 {
        self.inner
    }
}

impl FourDigit7SdOutput for FourDigit7Sd {
    fn as_raw(&self) -> u32 {
        let mut out: [u8; 4] = [0; 4];

        for (i, digit) in self.digits.iter().enumerate() {
            let digit_out: u8 = digit.into();
            out[i] = digit_out;
        }

        // i have absolutely no idea why this is necessary
        // we either shift an extra time in the pio program and have these offset, or we don't and have `FourDigit7SdRaw::as_raw()` offset
        // how ???
        out.rotate_left(1);

        u32::from_le_bytes(out)
    }
}

pub struct FourDigit7SdDriver {}

impl FourDigit7SdDriver {
    pub fn init<'a, ST, SU, DT, DU>(dma0: impl Channel, dma1: impl Channel, pio: &mut Pio<'a, PIO0>, segment_pins: UninitSerialPair<ST, SU>, digit_pins: UninitSerialPair<DT, DU>, segment_rclk: impl PioPin) -> Self
            where ST: PioPin, SU: PioPin, DT: PioPin, DU: PioPin
    {
        if unsafe { !INITIALIZED } {
            unsafe { INITIALIZED = true }

            setup_7sd_task_sm0(&mut pio.common, &mut pio.sm0, segment_pins, digit_pins, segment_rclk);
        
            pio.sm0.set_enable(true);
        
            init_ch(&dma0, &raw const HAHA, dma1.regs().al1_trans_count_trig().as_ptr(), DataSize::SIZE_BYTE, dma0.number(), true);
            init_ch(&dma1, &raw const OUTPUT, SM0_TXF0, DataSize::SIZE_WORD, dma0.number(), false);
            Self {}
        } else {
            panic!("Seven segment display driver already initialized");
        }
    }

    pub fn set_7sd(&mut self, output: impl FourDigit7SdOutput) {
        unsafe { OUTPUT = output.as_raw() }
    }
}