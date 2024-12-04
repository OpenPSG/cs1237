//! # CHIPSEA CS1237 ADC Driver
//!
//! This crate provides a Rust Driver For The
//! [CS1237](https://en.chipsea.com/product/details/?id=1155&pid=77) ADC.
//!
//! ## Example Usage
//!
//! ```rust
//! use embassy_stm32::gpio::{Level, Input, Output, Flex, Pull, Speed};
//! use embassy_stm32::exti::ExtiInput;
//! use embassy_stm32::{Peripheral, Peripherals};
//! use defmt::info;
//!
//! use cs1237::{CS1237, SampleRate, Gain, Channel};
//!
//! async fn print_adc_value(p: Peripherals) {
//! let clk = Output::new(p.PA0, Level::Low, Speed::VeryHigh);
//! let data_pin_clone = unsafe { p.PA1.clone_unchecked() };
//! let data = Flex::new(data_pin_clone);
//! let drdy = ExtiInput::new(Input::new(p.PA1, Pull::None), p.EXTI1);
//!
//! let mut adc = CS1237::new(
//!   clk,
//!   data,
//!   drdy,
//!   216_000_000,
//! );
//!
//! adc.set_config(SampleRate::Hz10, Gain::Gain1, Channel::Temperature).await.unwrap();
//!
//! let value = adc.read().await.unwrap();
//! info!("Value: {}", value);
//! }
//! ```
//!
//! ## License
//!
//! This crate is licensed under the Mozilla Public License 2.0 (MPL-2.0).
//! See the LICENSE file for more details.
//!

#![cfg_attr(not(test), no_std)]

use core::convert::TryFrom;
use embassy_stm32::{
    exti::ExtiInput,
    gpio::{Flex, Level, Output, Pin, Pull, Speed},
};
use embassy_time::{with_timeout, Duration};

#[cfg(feature = "defmt")]
use defmt::*;

/// The 2-wire serial interface clock frequency.
/// According to the datasheet, the maximum clock frequency is 1MHz.
/// We derate this a little to be safe.
const BUS_FREQUENCY_HZ: u32 = 500_000;

/// Sample rate configuration.
#[derive(Clone, Copy, Debug)]
pub enum SampleRate {
    Hz10 = 0,
    Hz40 = 1,
    Hz640 = 2,
    Hz1280 = 3,
}

impl TryFrom<i32> for SampleRate {
    type Error = &'static str;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            10 => Ok(SampleRate::Hz10),
            40 => Ok(SampleRate::Hz40),
            640 => Ok(SampleRate::Hz640),
            1280 => Ok(SampleRate::Hz1280),
            _ => Err("Invalid value for SampleRate"),
        }
    }
}

/// Gain configuration.
#[derive(Clone, Copy, Debug)]
pub enum Gain {
    Gain1 = 0,
    Gain2 = 1,
    Gain64 = 2,
    Gain128 = 3,
}

impl TryFrom<i32> for Gain {
    type Error = &'static str;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            10 => Ok(Gain::Gain1),
            2 => Ok(Gain::Gain2),
            64 => Ok(Gain::Gain64),
            128 => Ok(Gain::Gain128),
            _ => Err("Invalid value for Gain"),
        }
    }
}

/// ADC channel configuration.
#[derive(Clone, Copy, Debug)]
pub enum Channel {
    ChannelA = 0,
    Reserved = 1,
    Temperature = 2,
    InternalShort = 3,
}

impl TryFrom<i32> for Channel {
    type Error = &'static str;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Channel::ChannelA),
            1 => Ok(Channel::Reserved),
            2 => Ok(Channel::Temperature),
            3 => Ok(Channel::InternalShort),
            _ => Err("Invalid value for Channel"),
        }
    }
}

/// A driver for the CHIPSEA CS1237 ADC.
pub struct CS1237<CLK: Pin, DATA: Pin, DRDY: Pin> {
    clk: Output<'static, CLK>,
    data: Flex<'static, DATA>,
    drdy: ExtiInput<'static, DRDY>,
    cpu_freq_hz: u32,
}

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CS1237Error {
    NoTriggerPulse,
    SensorNotResponding,
}

impl<CLK, DATA, DRDY> CS1237<CLK, DATA, DRDY>
where
    CLK: Pin,
    DATA: Pin,
    DRDY: Pin,
{
    pub fn new(
        clk: Output<'static, CLK>,
        data: Flex<'static, DATA>,
        drdy: ExtiInput<'static, DRDY>,
        cpu_freq_hz: u32,
    ) -> Self {
        Self {
            clk,
            data,
            drdy,
            cpu_freq_hz,
        }
    }

    /// Read the next ADC sample.
    pub async fn read(&mut self) -> Result<i32, CS1237Error> {
        // Wait for a falling edge on the DATA/DRDY pin.
        self.data.set_as_input(Pull::None);

        let timeout = Duration::from_millis(110);
        with_timeout(timeout, self.drdy.wait_for_falling_edge())
            .await
            .map_err(|_| CS1237Error::NoTriggerPulse)?;

        let mut value: i32 = 0;
        for i in (0..24).rev() {
            value |= if self.read_bit() { 1 } else { 0 } << i;
        }

        // Fix the sign bit.
        if value > 0x7FFFFF {
            value -= 0x1000000;
        }

        Ok(value)
    }

    /// Set the sample rate, gain and channel configuration.
    pub async fn set_config(
        &mut self,
        sample_rate: SampleRate,
        gain: Gain,
        channel: Channel,
    ) -> Result<(), CS1237Error> {
        // Reset the chip so we are in a known state.
        self.power_down().await;
        self.power_up().await?;

        // Discard the 24bits of pending ADC data.
        for _ in 0..24 {
            self.read_bit();
        }

        // Get the write status bits (bits 25 and 26).
        let mut _write_status = if self.read_bit() { 1 } else { 0 };
        _write_status |= if self.read_bit() { 2 } else { 0 };

        // Indicate that a command follows (bits 27, 28, 29).
        self.read_bit();
        self.read_bit();
        self.read_bit();

        self.data.set_as_output(Speed::VeryHigh);

        // Write the configuration.
        let command: u8 = 0x65;
        for i in (0..7).rev() {
            self.write_bit((command >> i) & 0x1 != 0);
        }

        // write gap bit 37.
        self.write_bit(true);

        let config = ((sample_rate as u8) << 4) | ((gain as u8) << 2) | (channel as u8);

        for i in (0..8).rev() {
            self.write_bit((config >> i) & 0x1 != 0);
        }

        // final clock pulse, bit 46.
        self.data.set_as_input(Pull::None);
        self.read_bit();

        // Wait for the data line to go low, will take between 3ms and 300ms
        // Depending on configured sample rate.
        let timeout = Duration::from_millis(330);
        with_timeout(timeout, self.drdy.wait_for_falling_edge())
            .await
            .map_err(|_| CS1237Error::SensorNotResponding)?;

        Ok(())
    }

    /// Power down the ADC to save power.
    pub async fn power_down(&mut self) {
        self.clk.set_high();
        cortex_m::asm::delay(self.cpu_freq_hz / 10000); // 100us
    }

    /// Power up the ADC.
    pub async fn power_up(&mut self) -> Result<(), CS1237Error> {
        self.data.set_as_input(Pull::None);
        self.clk.set_low();

        // Wait for the data line to go low, will take between 3ms and 300ms
        // Depending on configured sample rate.
        let timeout = Duration::from_millis(330);
        with_timeout(timeout, self.drdy.wait_for_falling_edge())
            .await
            .map_err(|_| CS1237Error::SensorNotResponding)?;

        Ok(())
    }

    fn read_bit(&mut self) -> bool {
        self.clk.set_high();
        cortex_m::asm::delay(self.cpu_freq_hz / (2 * BUS_FREQUENCY_HZ));
        self.clk.set_low();
        cortex_m::asm::delay(self.cpu_freq_hz / (2 * BUS_FREQUENCY_HZ));
        self.data.is_high()
    }

    fn write_bit(&mut self, bit: bool) {
        self.data
            .set_level(if bit { Level::High } else { Level::Low });
        cortex_m::asm::delay(self.cpu_freq_hz / (4 * BUS_FREQUENCY_HZ));
        self.clk.set_high();
        cortex_m::asm::delay(self.cpu_freq_hz / (4 * BUS_FREQUENCY_HZ));
        self.clk.set_low();
        cortex_m::asm::delay(self.cpu_freq_hz / (2 * BUS_FREQUENCY_HZ));
    }
}
