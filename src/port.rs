use core::future::Future;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use seq_macro::seq;

use crate::config::{
    ConfigMode0, ConfigMode1, ConfigMode10, ConfigMode11, ConfigMode12, ConfigMode2, ConfigMode3,
    ConfigMode4, ConfigMode5, ConfigMode6, ConfigMode7, ConfigMode8, ConfigMode9, Port, ADCRANGE,
    AVR, DACRANGE, GPIMD, NSAMPLES, REG_ADC_DATA, REG_DAC_DATA,
};
use crate::{Error, Max11300};

pub struct Mode0Port<SPI: 'static, EN: 'static, M: RawMutex + 'static> {
    port: Port,
    max: &'static Mutex<M, Max11300<SPI, EN>>,
}

impl<SPI, EN, M> Mode0Port<SPI, EN, M>
where
    SPI: SpiBus,
    EN: OutputPin,
    M: RawMutex,
{
    pub(crate) fn new(port: Port, driver: &'static Mutex<M, Max11300<SPI, EN>>) -> Self {
        Self { port, max: driver }
    }
}

pub trait IntoConfiguredPort<CONFIG, SPI, EN, M, S, P>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    fn into_configured_port(
        self,
        config: CONFIG,
    ) -> impl Future<Output = Result<MaxPort<CONFIG, SPI, EN, M>, Error<S, P>>>;
}

seq!(N in 0..=12 {
    impl<SPI, EN, M, S, P> IntoConfiguredPort<ConfigMode~N, SPI, EN, M, S, P>
        for Mode0Port<SPI, EN, M>
    where
        SPI: SpiBus<Error = S> + 'static,
        EN: OutputPin<Error = P> + 'static,
        M: RawMutex + 'static
    {
        async fn into_configured_port(
            self,
            config: ConfigMode~N,
        ) -> Result<MaxPort<ConfigMode~N, SPI, EN, M>, Error<S, P>> {
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

pub struct MaxPort<CONFIG, SPI: 'static, EN: 'static, M: RawMutex + 'static> {
    config: CONFIG,
    port: Port,
    max: &'static Mutex<M, Max11300<SPI, EN>>,
}

pub trait IntoMode<CONFIG, SPI, EN, M, S, P>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    fn into_mode(
        self,
        config: CONFIG,
    ) -> impl Future<Output = Result<MaxPort<CONFIG, SPI, EN, M>, Error<S, P>>>;
}

seq!(N in 0..=12 {
    impl<CONFIG, SPI, EN, M, S, P> IntoMode<ConfigMode~N, SPI, EN, M, S, P>
        for MaxPort<CONFIG, SPI, EN, M>
    where
        SPI: SpiBus<Error = S> + 'static,
        EN: OutputPin<Error = P> + 'static,
        M: RawMutex + 'static,
    {
        async fn into_mode(
            self,
            config: ConfigMode~N,
        ) -> Result<MaxPort<ConfigMode~N, SPI, EN, M>, Error<S, P>> {
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

impl<SPI, EN, M, S, P> MaxPort<ConfigMode0, SPI, EN, M>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    // High impedance mode. Nothing can be done here.
}

impl<SPI, EN, M, S, P> MaxPort<ConfigMode1, SPI, EN, M>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    /// Configure the value of the interrupt threshold and edge detection on this port
    pub async fn configure_threshold(
        &mut self,
        threshold: u16,
        mode: GPIMD,
    ) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver
            ._gpi_configure_threshold(self.port, threshold, mode)
            .await
    }
}

impl<SPI, EN, M, S, P> MaxPort<ConfigMode3, SPI, EN, M>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    /// Configure the output level of the DAC when the port is set high
    pub async fn configure_level(&self, level: u16) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._gpo_configure_level(self.port, level).await
    }

    /// Set the port output high (to previously configured level)
    pub async fn set_high(&self) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._gpo_set_high(self.port).await
    }

    /// Set the port output low (zero)
    pub async fn set_low(&self) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._gpo_set_low(self.port).await
    }

    /// Toggle the port output
    pub async fn toggle(&self) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._gpo_toggle(self.port).await
    }
}

impl<SPI, EN, M, S, P> MaxPort<ConfigMode5, SPI, EN, M>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    /// Set the DAC output value
    pub async fn set_value(&self, data: u16) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._dac_set_value(self.port, data).await
    }

    /// Configure the analog voltage range for the DAC
    pub async fn configure_range(&mut self, range: DACRANGE) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.0 = range;
        driver
            ._configure_port(self.port, self.config.as_u16())
            .await
    }
}

impl<SPI, EN, M, S, P> MaxPort<ConfigMode6, SPI, EN, M>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    /// Set the DAC output value
    pub async fn set_value(&self, data: u16) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._dac_set_value(self.port, data).await
    }

    /// Configure the analog voltage reference for the ADC
    pub async fn configure_avr(&mut self, avr: AVR) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.0 = avr;
        driver
            ._configure_port(self.port, self.config.as_u16())
            .await
    }

    /// Configure the analog voltage range for the DAC
    pub async fn configure_range(&mut self, range: DACRANGE) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.1 = range;
        driver
            ._configure_port(self.port, self.config.as_u16())
            .await
    }

    /// Get the current value of the ADC (monitoring the DAC)
    pub async fn get_value(&self) -> Result<u16, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._adc_get_value(self.port).await
    }
}

impl<SPI, EN, M, S, P> MaxPort<ConfigMode7, SPI, EN, M>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    /// Get the current value of the ADC
    pub async fn get_value(&self) -> Result<u16, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver._adc_get_value(self.port).await
    }

    /// Configure the analog voltage reference for the ADC
    pub async fn configure_avr(&mut self, avr: AVR) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.0 = avr;
        driver
            ._configure_port(self.port, self.config.as_u16())
            .await
    }

    /// Configure the analog voltage range for the ADC
    pub async fn configure_range(&mut self, range: ADCRANGE) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.1 = range;
        driver
            ._configure_port(self.port, self.config.as_u16())
            .await
    }

    /// Configure the number of samples to take and average over
    pub async fn configure_nsamples(&mut self, nsamples: NSAMPLES) -> Result<(), Error<S, P>> {
        let mut driver = self.max.lock().await;
        self.config.2 = nsamples;
        driver
            ._configure_port(self.port, self.config.as_u16())
            .await
    }
}

pub struct Multiport<CONFIG, SPI: 'static, EN: 'static, M: RawMutex + 'static, const N: usize> {
    pub ports: [MaxPort<CONFIG, SPI, EN, M>; N],
}

impl<CONFIG, SPI, EN, M, S, P, const N: usize> Multiport<CONFIG, SPI, EN, M, N>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
{
    pub fn new(ports: [MaxPort<CONFIG, SPI, EN, M>; N]) -> Result<Self, Error<S, P>> {
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

impl<'a, SPI, EN, M, S, P, const N: usize> Multiport<ConfigMode5, SPI, EN, M, N>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
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

impl<'a, SPI, EN, M, S, P, const N: usize> Multiport<ConfigMode7, SPI, EN, M, N>
where
    SPI: SpiBus<Error = S> + 'static,
    EN: OutputPin<Error = P> + 'static,
    M: RawMutex + 'static,
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
