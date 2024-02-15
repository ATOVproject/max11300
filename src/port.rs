use core::future::Future;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use seq_macro::seq;

use crate::config::{
    ConfigMode0, ConfigMode1, ConfigMode10, ConfigMode11, ConfigMode12, ConfigMode2, ConfigMode3,
    ConfigMode4, ConfigMode5, ConfigMode6, ConfigMode7, ConfigMode8, ConfigMode9, Port, ADCRANGE,
    AVR, DACRANGE, GPIMD, NSAMPLES, REG_ADC_DATA, REG_DAC_DATA, REG_GPO_DATA, REG_IRQ_MODE,
};
use crate::{Error, WrappedDriver};

pub struct Mode0Port<'a, SPI, EN> {
    port: Port,
    max: &'a WrappedDriver<SPI, EN>,
}

impl<'a, SPI, EN> Mode0Port<'a, SPI, EN>
where
    SPI: SpiBus,
    EN: OutputPin,
{
    pub(crate) fn new(port: Port, driver: &'a WrappedDriver<SPI, EN>) -> Self {
        Self { port, max: driver }
    }
}

pub trait IntoConfiguredPort<'a, CONFIG, SPI: 'a, EN: 'a, S, P> {
    fn into_configured_port(
        self,
        config: CONFIG,
    ) -> impl Future<Output = Result<MaxPort<'a, CONFIG, SPI, EN>, Error<S, P>>>;
}

seq!(N in 0..=12 {
    impl<'a, SPI, EN, S, P> IntoConfiguredPort<'a, ConfigMode~N, SPI, EN, S, P>
        for Mode0Port<'a, SPI, EN>
    where
        SPI: SpiBus<Error = S> + 'a,
        EN: OutputPin<Error = P>,
    {
        async fn into_configured_port(
            self,
            config: ConfigMode~N,
        ) -> Result<MaxPort<'a, ConfigMode~N, SPI, EN>, Error<S, P>> {
            let mut locked_max = self.max.lock().await;
            locked_max.write_register(self.port.as_config_addr(), config.as_u16()).await?;
            Ok(MaxPort {
                config,
                port: self.port,
                max: self.max,
            })
        }
    }
});

pub struct MaxPort<'a, CONFIG, SPI, EN> {
    config: CONFIG,
    port: Port,
    max: &'a WrappedDriver<SPI, EN>,
}

pub trait IntoMode<'a, CONFIG, SPI: 'a, EN: 'a, S, P> {
    fn into_mode(
        self,
        config: CONFIG,
    ) -> impl Future<Output = Result<MaxPort<'a, CONFIG, SPI, EN>, Error<S, P>>>;
}

seq!(N in 0..=12 {
    impl<'a, CONFIG, SPI, EN, S, P> IntoMode<'a, ConfigMode~N, SPI, EN, S, P>
        for MaxPort<'a, CONFIG, SPI, EN>
    where
        SPI: SpiBus<Error = S> + 'a,
        EN: OutputPin<Error = P>,
    {
        async fn into_mode(
            self,
            config: ConfigMode~N,
        ) -> Result<MaxPort<'a, ConfigMode~N, SPI, EN>, Error<S, P>> {
            let mut driver = self.max.lock().await;
            driver.write_register(self.port.as_config_addr(), config.as_u16()).await?;
            Ok(MaxPort {
                config,
                port: self.port,
                max: self.max,
            })
        }
    }
});

impl<'a, SPI, EN, S, P> MaxPort<'a, ConfigMode0, SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    // High impedance mode. Nothing can be done here.
}

impl<'a, SPI, EN, S, P> MaxPort<'a, ConfigMode1, SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Configure the value of the interrupt threshold and edge detection on this port
    pub async fn configure_threshold(
        &mut self,
        threshold: u16,
        mode: GPIMD,
    ) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        // Set IRQ mode for port
        let pos = self.port as usize % 8 * 2;
        let mut current = driver
            .read_register(REG_IRQ_MODE + (self.port as u8 / 3))
            .await?;
        let mut next = (mode as u16) << pos;
        current &= !(0b11 << pos);
        next |= current;
        driver
            .write_register(REG_IRQ_MODE + (self.port as u8 / 3), next)
            .await?;
        // Set threshold for port
        driver
            .write_register(REG_DAC_DATA + (self.port as u8), threshold)
            .await
    }
}

impl<'a, SPI, EN, S, P> MaxPort<'a, ConfigMode3, SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Configure the output level of the DAC when the port is set high
    pub async fn configure_level(&self, level: u16) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver
            .write_register(REG_DAC_DATA + (self.port as u8), level)
            .await
    }

    /// Set the port output high (to previously configured level)
    pub async fn set_high(&self) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        let current = driver
            .read_register(REG_GPO_DATA + (self.port as u8 / 2))
            .await?;
        let pos = self.port as usize % 16;
        let next = current | (1 << pos);
        driver
            .write_register(REG_GPO_DATA + (self.port as u8), next)
            .await
    }

    /// Set the port output low (zero)
    pub async fn set_low(&self) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        let current = driver
            .read_register(REG_GPO_DATA + (self.port as u8 / 2))
            .await?;
        let pos = self.port as usize % 16;
        let next = current & !(1 << pos);
        driver
            .write_register(REG_GPO_DATA + (self.port as u8), next)
            .await
    }

    /// Toggle the port output
    pub async fn toggle(&self) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        let current = driver
            .read_register(REG_GPO_DATA + (self.port as u8 / 2))
            .await?;
        let pos = self.port as usize % 16;
        let next = current ^ (1 << pos);
        driver
            .write_register(REG_GPO_DATA + (self.port as u8), next)
            .await
    }
}

impl<'a, SPI, EN, S, P> MaxPort<'a, ConfigMode5, SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Set the DAC output value
    pub async fn set_value(&self, data: u16) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver
            .write_register(REG_DAC_DATA + (self.port as u8), data)
            .await
    }

    /// Configure the analog voltage range for the DAC
    pub async fn configure_range(&mut self, range: DACRANGE) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.0 = range;
        driver
            .write_register(self.port.as_config_addr(), self.config.as_u16())
            .await
    }
}

impl<'a, SPI, EN, S, P> MaxPort<'a, ConfigMode6, SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Set the DAC output value
    pub async fn set_value(&self, data: u16) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver
            .write_register(REG_DAC_DATA + (self.port as u8), data)
            .await
    }

    /// Configure the analog voltage reference for the ADC
    pub async fn configure_avr(&mut self, avr: AVR) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.0 = avr;
        driver
            .write_register(self.port.as_config_addr(), self.config.as_u16())
            .await
    }

    /// Configure the analog voltage range for the DAC
    pub async fn configure_range(&mut self, range: DACRANGE) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.1 = range;
        driver
            .write_register(self.port.as_config_addr(), self.config.as_u16())
            .await
    }

    /// Get the current value of the ADC (monitoring the DAC)
    pub async fn get_value(&self) -> Result<u16, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.read_register(REG_ADC_DATA + (self.port as u8)).await
    }
}

impl<'a, SPI, EN, S, P> MaxPort<'a, ConfigMode7, SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Get the current value of the ADC
    pub async fn get_value(&self) -> Result<u16, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.read_register(REG_ADC_DATA + (self.port as u8)).await
    }

    /// Configure the analog voltage reference for the ADC
    pub async fn configure_avr(&mut self, avr: AVR) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.0 = avr;
        driver
            .write_register(self.port.as_config_addr(), self.config.as_u16())
            .await
    }

    /// Configure the analog voltage range for the ADC
    pub async fn configure_range(&mut self, range: ADCRANGE) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.1 = range;
        driver
            .write_register(self.port.as_config_addr(), self.config.as_u16())
            .await
    }

    /// Configure the number of samples to take and average over
    pub async fn configure_nsamples(&mut self, nsamples: NSAMPLES) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.2 = nsamples;
        driver
            .write_register(self.port.as_config_addr(), self.config.as_u16())
            .await
    }
}

pub struct Multiport<'a, CONFIG, SPI, EN, const N: usize> {
    pub ports: [MaxPort<'a, CONFIG, SPI, EN>; N],
}

impl<'a, CONFIG, SPI, EN, S, P, const N: usize> Multiport<'a, CONFIG, SPI, EN, N>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    pub fn new(ports: [MaxPort<'a, CONFIG, SPI, EN>; N]) -> Result<Self, Error<S, P>> {
        // Check if all the ports are in a row
        // We might weaken this requirement in the future and use the context based burst mode
        for neighbours in ports.windows(2) {
            if neighbours[1].port as u8 != (neighbours[0].port as u8) + 1 {
                return Err(Error::Port);
            }
        }
        Ok(Self { ports })
    }
}

impl<'a, SPI, EN, S, P, const N: usize> Multiport<'a, ConfigMode5, SPI, EN, N>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Set the DAC output values for all owned ports
    pub async fn set_values(&mut self, data: &[u16; N]) -> Result<(), Error<S, P>> {
        // We're just using the driver of the first port here
        let mut driver = self.ports[0].max.lock().await;
        driver
            .write_registers(REG_DAC_DATA + (self.ports[0].port as u8), data)
            .await
    }
}

impl<'a, SPI, EN, S, P, const N: usize> Multiport<'a, ConfigMode7, SPI, EN, N>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Get the current values of the ADC for all owned ports
    pub async fn get_values(&mut self, buf: &'a mut [u16; N]) -> Result<&'a [u16], Error<S, P>> {
        let mut driver = self.ports[0].max.lock().await;
        driver
            .read_registers(REG_ADC_DATA + (self.ports[0].port as u8), buf)
            .await
    }
}

// TODO: implement mode 4, 8, 9, 10, 11, 12 as individual structs that take Mode0Ports
