use core::convert::Infallible;
use embassy_stm32::exti::{Channel, ExtiInput};
use embassy_stm32::gpio::{Flex, Input, Pin, Pull, Speed};
use embassy_stm32::Peripheral;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin};
use embedded_hal_async::digital::Wait;

/// A meta-pin that combines a GPIO pin with an EXTI input.
/// This is required for the CHIPSEA CS1237 ADC driver.
pub struct MetaPin<'a, P: Pin> {
    pin: Flex<'a, P>,
    interrupt: ExtiInput<'a, P>,
}

impl<'a, P: Pin> MetaPin<'a, P> {
    /// Create a new meta-pin from a GPIO pin and an EXTI channel.
    pub fn new(
        pin: P,
        interrupt_channel: impl Channel + Peripheral<P = P::ExtiChannel> + 'a,
    ) -> Self {
        Self {
            pin: Flex::new(unsafe { pin.clone_unchecked() }),
            interrupt: ExtiInput::new(Input::new(pin, Pull::None), interrupt_channel),
        }
    }
}

impl<'a, P: Pin> ErrorType for MetaPin<'a, P> {
    type Error = Infallible;
}

impl<'a, P: Pin> crate::FlexPin for MetaPin<'a, P> {
    fn set_as_input(&mut self) -> Result<(), Self::Error> {
        self.pin.set_as_input(Pull::None);
        Ok(())
    }

    fn set_as_output(&mut self) -> Result<(), Self::Error> {
        self.pin.set_as_output(Speed::VeryHigh);
        Ok(())
    }
}

impl<'a, P: Pin> OutputPin for MetaPin<'a, P> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }
}

impl<'a, P: Pin> InputPin for MetaPin<'a, P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_high())
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.pin.is_low())
    }
}

impl<'a, P: Pin> Wait for MetaPin<'a, P> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.interrupt.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.interrupt.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.interrupt.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.interrupt.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.interrupt.wait_for_any_edge().await;
        Ok(())
    }
}
