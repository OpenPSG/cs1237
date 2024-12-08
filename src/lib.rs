//! # CHIPSEA CS1237 ADC Driver
//!
//! An embedded_hal_async driver for the
//! [CS1237](https://en.chipsea.com/product/details/?id=1155&pid=77) ADC.
//!
//! ## Example Usage
//!
//! ```no_run
//! use embassy_time::Delay;
//! use cs1237::{Cs1237, Gain, SamplesPerSecond, Channel};
//! use cs1237::stm32::MetaPin;
//! use embassy_stm32::{gpio::{Output, Level, Speed}, Peripherals, peripherals};
//!
//! async fn read_adc_value(p: Peripherals) {
//!   let clk_pin = Output::new(p.PA0, Level::Low, Speed::VeryHigh);
//!   let data_pin = MetaPin::new(p.PA1, p.EXTI1);
//!
//!   let mut adc: Cs1237<Output<'static, peripherals::PA0>, MetaPin<'static, peripherals::PA1>, Delay> = Cs1237::try_new(
//!     clk_pin,
//!     data_pin,
//!     Delay,
//!     Gain::G1,
//!     SamplesPerSecond::SPS10,
//!     Channel::Temperature
//!   ).await.unwrap();
//!
//!   let _value = adc.read().await.unwrap();
//! }
//! ```
//!
//! ## License
//!
//! This crate is licensed under the Mozilla Public License 2.0 (MPL-2.0).
//! See the LICENSE file for more details.

#![cfg_attr(not(test), no_std)]

use byteorder::{BigEndian, ByteOrder};
use embassy_time::{with_timeout, Duration};
use embedded_hal::digital::{InputPin, OutputPin, PinState};
use embedded_hal_async::digital::Wait;

#[cfg(feature = "embassy-stm32")]
pub mod stm32;

/// The 2-wire serial interface clock frequency.
/// According to the datasheet, the maximum clock frequency is 1MHz.
/// We derate this a little to be safe.
pub const DEFAULT_BUS_FREQUENCY_HZ: usize = 500_000;

/// Sample rate configuration.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum SamplesPerSecond {
    SPS10 = 0,
    SPS40 = 1,
    SPS640 = 2,
    SPS1280 = 3,
}

/// Gain configuration.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum Gain {
    G1 = 0,
    G2 = 1,
    G64 = 2,
    G128 = 3,
}

/// ADC channel configuration.
#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum Channel {
    ChannelA = 0,
    Reserved = 1,
    Temperature = 2,
    InternalShort = 3,
}

/// A driver for the CHIPSEA CS1237 ADC.
pub struct Cs1237<
    ClockPin,
    DataPin,
    Delay,
    const BUS_FREQUENCY_HZ: usize = DEFAULT_BUS_FREQUENCY_HZ,
> where
    ClockPin: OutputPin,
    DataPin: InputPin + OutputPin + FlexPin + Wait,
    Delay: embedded_hal::delay::DelayNs,
{
    clk_pin: ClockPin,
    data_pin: DataPin,
    delay: Delay,
    bus_period_ns: u32,
}

/// Errors that can occur when using the CS1237 driver.
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<PinErr> {
    Pin(PinErr),
    NoTriggerPulse,
    SensorNotResponding,
}

impl<ClockPin, DataPin, Delay, PinErr, const BUS_FREQUENCY_HZ: usize>
    Cs1237<ClockPin, DataPin, Delay, BUS_FREQUENCY_HZ>
where
    ClockPin: OutputPin<Error = PinErr>,
    DataPin: InputPin<Error = PinErr>
        + OutputPin<Error = PinErr>
        + FlexPin<Error = PinErr>
        + Wait<Error = PinErr>,
    Delay: embedded_hal::delay::DelayNs,
{
    /// Attempt to create a new CS1237 driver.
    pub async fn try_new(
        clk_pin: ClockPin,
        data_pin: DataPin,
        delay: Delay,
        gain: Gain,
        sample_rate: SamplesPerSecond,
        channel: Channel,
    ) -> Result<Self, Error<PinErr>> {
        let mut adc = Self {
            bus_period_ns: 1_000_000_000 / BUS_FREQUENCY_HZ as u32,
            clk_pin,
            data_pin,
            delay,
        };
        adc.update_config(gain, sample_rate, channel).await?;
        Ok(adc)
    }

    /// Read the next ADC sample.
    pub async fn read(&mut self) -> Result<i32, Error<PinErr>> {
        // Wait for a falling edge on the DATA/DRDY pin.
        self.data_pin.set_as_input().map_err(Error::Pin)?;

        let timeout = Duration::from_millis(110);
        with_timeout(timeout, self.data_pin.wait_for_falling_edge())
            .await
            .map_err(|_| Error::NoTriggerPulse)?
            .map_err(Error::Pin)?;

        // Read the 24bit sample.
        let mut buf = [0u8; 3];
        critical_section::with(|_| -> Result<(), Error<PinErr>> {
            for byte in buf.iter_mut() {
                *byte = self.read_byte()?;
            }
            Ok(())
        })?;

        Ok(BigEndian::read_i24(&buf))
    }

    /// Set the sample rate, gain and channel configuration.
    pub async fn update_config(
        &mut self,
        gain: Gain,
        sample_rate: SamplesPerSecond,
        channel: Channel,
    ) -> Result<(), Error<PinErr>> {
        // Reset the chip so we are in a known state.
        self.power_down().await?;
        self.power_up().await?;

        critical_section::with(|_| -> Result<(), Error<PinErr>> {
            // Discard the 24bits of pending ADC data.
            for _ in 0..24 {
                self.read_bit()?;
            }

            // Get the write status bits (bits 25 and 26).
            let mut _write_status = if self.read_bit()? { 1 } else { 0 };
            _write_status |= if self.read_bit()? { 2 } else { 0 };

            // Indicate that a command follows (bits 27, 28, 29).
            self.read_bit()?;
            self.read_bit()?;
            self.read_bit()?;

            self.data_pin.set_as_output().map_err(Error::Pin)?;

            // Write the configuration.
            let command: u8 = 0x65;
            for i in (0..7).rev() {
                self.write_bit((command >> i) & 0x1 != 0)?;
            }

            // write gap bit 37.
            self.write_bit(true)?;

            let config = ((sample_rate as u8) << 4) | ((gain as u8) << 2) | (channel as u8);
            for i in (0..8).rev() {
                self.write_bit((config >> i) & 0x1 != 0)?;
            }

            // Final clock pulse, bit 46.
            self.data_pin.set_as_input().map_err(Error::Pin)?;
            self.read_bit()?;

            Ok(())
        })?;

        // Wait for the data line to go low, will take between 3ms and 300ms
        // Depending on configured sample rate.
        let timeout = Duration::from_millis(330);
        with_timeout(timeout, self.data_pin.wait_for_falling_edge())
            .await
            .map_err(|_| Error::SensorNotResponding)?
            .map_err(Error::Pin)?;

        Ok(())
    }

    /// Power down the ADC to save power.
    pub async fn power_down(&mut self) -> Result<(), Error<PinErr>> {
        self.clk_pin.set_high().map_err(Error::Pin)?;
        self.delay.delay_us(100);
        Ok(())
    }

    /// Power up the ADC.
    pub async fn power_up(&mut self) -> Result<(), Error<PinErr>> {
        self.data_pin.set_as_input().map_err(Error::Pin)?;
        self.clk_pin.set_low().map_err(Error::Pin)?;

        // Wait for the data line to go low, will take between 3ms and 300ms
        // Depending on configured sample rate.
        let timeout = Duration::from_millis(330);
        with_timeout(timeout, self.data_pin.wait_for_falling_edge())
            .await
            .map_err(|_| Error::SensorNotResponding)?
            .map_err(Error::Pin)?;

        Ok(())
    }

    fn read_byte(&mut self) -> Result<u8, Error<PinErr>> {
        let mut byte = 0;
        for _ in (0..8).rev() {
            byte <<= 1;
            byte |= if self.read_bit()? { 1 } else { 0 };
        }
        Ok(byte)
    }

    fn read_bit(&mut self) -> Result<bool, Error<PinErr>> {
        self.clk_pin.set_high().map_err(Error::Pin)?;
        self.delay.delay_ns(self.bus_period_ns / 2);
        self.clk_pin.set_low().map_err(Error::Pin)?;
        self.delay.delay_ns(self.bus_period_ns / 2);
        self.data_pin.is_high().map_err(Error::Pin)
    }

    fn write_bit(&mut self, bit: bool) -> Result<(), Error<PinErr>> {
        self.data_pin
            .set_state(if bit { PinState::High } else { PinState::Low })
            .map_err(Error::Pin)?;
        self.clk_pin.set_high().map_err(Error::Pin)?;
        self.delay.delay_ns(self.bus_period_ns / 2);
        self.clk_pin.set_low().map_err(Error::Pin)?;
        self.delay.delay_ns(self.bus_period_ns / 2);
        Ok(())
    }
}

/// FlexPin is a pin that can be set as input or output.
/// This is required because the CS1237 uses a single pin for data input,
/// output, and interrupts.
pub trait FlexPin: embedded_hal::digital::ErrorType {
    /// Set the pin as an input.
    fn set_as_input(&mut self) -> Result<(), Self::Error>;

    /// Set the pin as an output.
    fn set_as_output(&mut self) -> Result<(), Self::Error>;
}
