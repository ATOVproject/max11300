#![cfg_attr(not(test), no_std)]

pub mod config;
mod port;

use core::future::Future;

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use heapless::Vec;
use seq_macro::seq;

use config::*;

pub use port::{IntoConfiguredPort, IntoMode};
pub use port::{MaxPort, Mode0Port, Multiport};

#[derive(Debug)]
pub enum Error<S, P> {
    /// SPI bus error
    Spi(S),
    /// CS pin error
    Pin(P),
    /// Connection error (device not found)
    Conn,
    /// Address error (invalid or out of bounds)
    Address,
    /// Port error (invalid or out of bounds)
    Port,
    /// Mode error (wrong port mode for method)
    Mode,
}

pub trait ConfigurePort<CONFIG, S, P> {
    fn configure_port(
        &mut self,
        port: Port,
        config: CONFIG,
    ) -> impl Future<Output = Result<(), Error<S, P>>>;
}

/// Max11300 driver
pub struct Max11300<SPI, EN> {
    channel_config: [u16; 20],
    config: DeviceConfig,
    enable: EN,
    spi: SPI,
}

impl<SPI, EN, S, P> Max11300<SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Create a new Max11300 instance
    pub async fn try_new(
        spi: SPI,
        mut enable: EN,
        config: DeviceConfig,
    ) -> Result<Self, Error<S, P>> {
        enable.set_high().map_err(Error::Pin)?;
        let mut max = Self {
            channel_config: [0; 20],
            config,
            enable,
            spi,
        };
        if max.read_register(REG_DEVICE_ID).await? != DEVICE_ID {
            return Err(Error::Conn);
        }
        max.write_register(REG_DEVICE_CTRL, config.as_u16()).await?;
        Ok(max)
    }

    /// Return the current DeviceConfig
    pub fn config(&self) -> DeviceConfig {
        self.config
    }

    /// Configure the input threshold for a ConfigMode1 Port
    pub async fn gpi_configure_threshold(
        &mut self,
        port: Port,
        threshold: u16,
        mode: GPIMD,
    ) -> Result<(), Error<S, P>> {
        if self.get_mode(port) != 1 {
            return Err(Error::Mode);
        }
        self._gpi_configure_threshold(port, threshold, mode).await
    }

    /// Configure the digital output level for a ConfigMode3 Port
    pub async fn gpo_configure_level(&mut self, port: Port, level: u16) -> Result<(), Error<S, P>> {
        if self.get_mode(port) != 3 {
            return Err(Error::Mode);
        }
        self._gpo_configure_level(port, level).await
    }

    /// Set a digital output high on a ConfigMode3 Port
    pub async fn gpo_set_high(&mut self, port: Port) -> Result<(), Error<S, P>> {
        if self.get_mode(port) != 3 {
            return Err(Error::Mode);
        }
        self._gpo_set_high(port).await
    }

    /// Set a digital output low on a ConfigMode3 Port
    pub async fn gpo_set_low(&mut self, port: Port) -> Result<(), Error<S, P>> {
        if self.get_mode(port) != 3 {
            return Err(Error::Mode);
        }
        self._gpo_set_low(port).await
    }

    /// Toggle digital output on a ConfigMode3 Port
    pub async fn gpo_toggle(&mut self, port: Port) -> Result<(), Error<S, P>> {
        if self.get_mode(port) != 3 {
            return Err(Error::Mode);
        }
        self._gpo_toggle(port).await
    }

    /// Set a DAC value for a ConfigMode5 Port
    pub async fn dac_set_value(&mut self, port: Port, data: u16) -> Result<(), Error<S, P>> {
        if self.get_mode(port) != 5 {
            return Err(Error::Mode);
        }
        self._dac_set_value(port, data).await
    }

    /// Read an ADC value from a ConfigMode7 Port
    pub async fn adc_get_value(&mut self, port: Port) -> Result<u16, Error<S, P>> {
        if self.get_mode(port) != 7 {
            return Err(Error::Mode);
        }
        self._adc_get_value(port).await
    }

    /// Read the Interrupts from the Interrupt register
    pub async fn int_read(&mut self) -> Result<Interrupts, Error<S, P>> {
        let val = self.read_register(REG_INTERRUPT).await?;
        Ok(Interrupts::from(val))
    }

    /// Read the raw Interrupts from the Interrupt register
    pub async fn int_read_raw(&mut self) -> Result<u16, Error<S, P>> {
        let val = self.read_register(REG_INTERRUPT).await?;
        Ok(val)
    }

    /// Read the GPI Status Interrupts from the GPIST register
    pub async fn int_read_gpist(&mut self) -> Result<u32, Error<S, P>> {
        let mut data = [0_u16; 2];
        self.read_registers(REG_GPI_STATUS, &mut data).await?;
        Ok((data[0] as u32) << 16 | data[1] as u32)
    }

    /// Get the mode number X (ConfigModeX) for a Port
    pub fn get_mode(&self, port: Port) -> u8 {
        ((self.channel_config[port.as_usize()] >> 12) & 0xf) as u8
    }

    async fn read_register(&mut self, address: u8) -> Result<u16, Error<S, P>> {
        if address > MAX_ADDRESS {
            return Err(Error::Address);
        }
        let mut buf = [0, 0, 0];
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi
            .transfer(&mut buf, &[address << 1 | 1])
            .await
            .map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok((buf[1] as u16) << 8 | buf[2] as u16)
    }

    async fn read_registers<'a>(
        &mut self,
        start_address: u8,
        data: &'a mut [u16],
    ) -> Result<&'a [u16], Error<S, P>> {
        if data.len() > 20 || start_address + data.len() as u8 - 1 > MAX_ADDRESS {
            return Err(Error::Address);
        }
        // 2x20 data bytes maximum
        let mut buf: Vec<u8, 40> = Vec::new();
        // Actual size of u16 output buffer times two plus address byte
        buf.resize(data.len() * 2, 0).ok();
        // Read values into buf
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi
            .transfer(&mut buf, &[start_address << 1 | 1])
            .await
            .map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        // Copy to data buffer
        for (i, bytes) in buf[1..].chunks(2).enumerate() {
            data[i] = (bytes[0] as u16) << 8 | bytes[1] as u16;
        }
        Ok(data)
    }

    async fn write_register(&mut self, address: u8, data: u16) -> Result<(), Error<S, P>> {
        if address > MAX_ADDRESS {
            return Err(Error::Address);
        }
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi
            .write(&[address << 1, (data >> 8) as u8, (data & 0xff) as u8])
            .await
            .map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    async fn write_registers(
        &mut self,
        start_address: u8,
        data: &[u16],
    ) -> Result<(), Error<S, P>> {
        if data.len() > 20 || start_address + data.len() as u8 - 1 > MAX_ADDRESS {
            return Err(Error::Address);
        }
        // 1 address byte, 2x20 data bytes maximum
        let mut buf: Vec<u8, 41> = Vec::new();
        // Actual size of u16 data buffer times two plus address byte
        buf.resize(data.len() * 2 + 1, 0).ok();
        // Write instruction
        buf[0] = start_address << 1;
        for (i, &data_u16) in data.iter().enumerate() {
            buf[i * 2 + 1] = (data_u16 >> 8) as u8;
            buf[i * 2 + 2] = (data_u16 & 0xff) as u8;
        }
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi.write(&buf).await.map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    async fn _configure_port(&mut self, port: Port, config: u16) -> Result<(), Error<S, P>> {
        self.write_register(port.as_config_addr(), config).await
    }

    async fn _gpi_configure_threshold(
        &mut self,
        port: Port,
        threshold: u16,
        mode: GPIMD,
    ) -> Result<(), Error<S, P>> {
        // Set IRQ mode for port
        let pos = port as usize % 8 * 2;
        let mut current = self.read_register(REG_IRQ_MODE + (port as u8 / 3)).await?;
        let mut next = (mode as u16) << pos;
        current &= !(0b11 << pos);
        next |= current;
        self.write_register(REG_IRQ_MODE + (port as u8 / 3), next)
            .await?;
        // Set threshold for port
        self.write_register(REG_DAC_DATA + (port as u8), threshold)
            .await
    }

    async fn _gpo_configure_level(&mut self, port: Port, level: u16) -> Result<(), Error<S, P>> {
        self.write_register(REG_DAC_DATA + (port as u8), level)
            .await
    }

    async fn _gpo_set_high(&mut self, port: Port) -> Result<(), Error<S, P>> {
        let current = self.read_register(REG_GPO_DATA + (port as u8 / 2)).await?;
        let pos = port as usize % 16;
        let next = current | (1 << pos);
        self.write_register(REG_GPO_DATA + (port as u8), next).await
    }

    async fn _gpo_set_low(&mut self, port: Port) -> Result<(), Error<S, P>> {
        let current = self.read_register(REG_GPO_DATA + (port as u8 / 2)).await?;
        let pos = port as usize % 16;
        let next = current & !(1 << pos);
        self.write_register(REG_GPO_DATA + (port as u8), next).await
    }

    async fn _gpo_toggle(&mut self, port: Port) -> Result<(), Error<S, P>> {
        let current = self.read_register(REG_GPO_DATA + (port as u8 / 2)).await?;
        let pos = port as usize % 16;
        let next = current ^ (1 << pos);
        self.write_register(REG_GPO_DATA + (port as u8), next).await
    }

    async fn _dac_set_value(&mut self, port: Port, data: u16) -> Result<(), Error<S, P>> {
        self.write_register(REG_DAC_DATA + (port as u8), data).await
    }

    async fn _adc_get_value(&mut self, port: Port) -> Result<u16, Error<S, P>> {
        self.read_register(REG_ADC_DATA + (port as u8)).await
    }
}

seq!(N in 0..=12 {
    impl<SPI, EN, S, P> ConfigurePort<ConfigMode~N, S, P> for Max11300<SPI, EN>
    where
        SPI: SpiBus<Error = S>,
        EN: OutputPin<Error = P>,
    {
        async fn configure_port(&mut self, port: Port, config: ConfigMode~N) -> Result<(), Error<S, P>> {
            let cfg = config.as_u16();
            self._configure_port(port, config.as_u16()).await?;
            self.channel_config[port.as_usize()] = cfg;
            Ok(())
        }
    }
});

pub struct ReadInterrupts<SPI: 'static, EN: 'static, M: RawMutex + 'static> {
    max: &'static Mutex<M, Max11300<SPI, EN>>,
}

impl<SPI, EN, M, S, P> ReadInterrupts<SPI, EN, M>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
    M: RawMutex,
{
    pub fn new(max: &'static Mutex<M, Max11300<SPI, EN>>) -> Self {
        Self { max }
    }

    /// Read the Interrupts from the Interrupt register
    pub async fn read(&self) -> Result<Interrupts, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.int_read().await
    }

    /// Read the raw Interrupts from the Interrupt register
    pub async fn read_raw(&self) -> Result<u16, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.int_read_raw().await
    }

    /// Read the GPI Status Interrupts from the GPIST register
    pub async fn read_gpist(&self) -> Result<u32, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.int_read_gpist().await
    }
}

seq!(N in 0..20 {
    pub struct Ports<SPI: 'static, EN: 'static, M: RawMutex + 'static> {
        pub interrupts: ReadInterrupts<SPI, EN, M>,
        #(
            pub port~N: Mode0Port<SPI, EN, M>,
        )*
    }
});

seq!(N in 0..20 {
    impl<SPI, EN, M> Ports<SPI, EN, M>
    where
        SPI: SpiBus,
        EN: OutputPin,
        M: RawMutex,
    {
        pub fn new(max: &'static Mutex<M, Max11300<SPI, EN>>) -> Self {
            Self {
                interrupts: ReadInterrupts::new(max),
                #(
                    port~N: Mode0Port::new(Port::P~N, max),
                )*
            }
        }
    }
});

#[cfg(test)]
mod tests {
    use crate::{
        config::{ConfigMode1, ConfigMode5, ConfigMode7, ADCRANGE, AVR, DACRANGE, NSAMPLES},
        port::{IntoConfiguredPort, IntoMode},
    };

    use super::*;
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};
    use embedded_hal_mock::{
        common::Generic,
        eh1::digital::{Mock as PinMock, State as PinState, Transaction as PinTransaction},
    };
    use static_cell::StaticCell;

    type MockMax = Max11300<Generic<SpiTransaction<u8>>, Generic<PinTransaction>>;

    static MAX0: StaticCell<Mutex<NoopRawMutex, MockMax>> = StaticCell::new();
    static MAX1: StaticCell<Mutex<NoopRawMutex, MockMax>> = StaticCell::new();
    static MAX2: StaticCell<Mutex<NoopRawMutex, MockMax>> = StaticCell::new();
    static MAX3: StaticCell<Mutex<NoopRawMutex, MockMax>> = StaticCell::new();

    #[tokio::test]
    async fn into_configured() {
        let config = DeviceConfig::default();
        let pin_expectations = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ];
        let spi_expectations = [
            // connection check
            SpiTransaction::transfer(vec![1], vec![0x0, 0x04, 0x24]),
            // write default configuration
            SpiTransaction::write_vec(vec![0x10 << 1, 0, 0]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let _max = Max11300::try_new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();
        pin.done();
        spi.done();
    }

    #[tokio::test]
    async fn config_modes() {
        let config = DeviceConfig::default();
        let pin_expectations = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ];
        let spi_expectations = [
            // connection check
            SpiTransaction::transfer(vec![1], vec![0x0, 0x04, 0x24]),
            // write default configuration
            SpiTransaction::write_vec(vec![0x10 << 1, 0, 0]),
            // configure port
            SpiTransaction::write_vec(vec![0x25 << 1, 16, 0]),
            // reconfigure port
            SpiTransaction::write_vec(vec![0x25 << 1, 113, 192]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let max = Max11300::try_new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();

        let max = MAX0.init(Mutex::new(max));
        let ports = Ports::new(max);

        // Configure port 5 for the first time
        let port5 = ports.port5.into_configured_port(ConfigMode1).await.unwrap();
        // Reconfigure port 5
        port5
            .into_mode(ConfigMode7(
                AVR::InternalRef,
                ADCRANGE::Rg0_10v,
                NSAMPLES::Samples64,
            ))
            .await
            .unwrap();
        pin.done();
        spi.done();
    }

    #[tokio::test]
    async fn config_mode_5() {
        let config = DeviceConfig::default();
        let pin_expectations = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ];
        let spi_expectations = [
            // connection check
            SpiTransaction::transfer(vec![1], vec![0x0, 0x04, 0x24]),
            // write default configuration
            SpiTransaction::write_vec(vec![0x10 << 1, 0, 0]),
            // configure port
            SpiTransaction::write_vec(vec![0x25 << 1, 81, 0]),
            // set value on port
            SpiTransaction::write_vec(vec![0x65 << 1, 0, 42]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let max = Max11300::try_new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();

        let max = MAX1.init(Mutex::new(max));
        let ports = Ports::new(max);

        let port5 = ports
            .port5
            .into_configured_port(ConfigMode5(DACRANGE::Rg0_10v))
            .await
            .unwrap();
        port5.set_value(42).await.unwrap();
        pin.done();
        spi.done();
    }

    #[tokio::test]
    async fn config_mode_7() {
        let config = DeviceConfig::default();
        let pin_expectations = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ];
        let spi_expectations = [
            // connection check
            SpiTransaction::transfer(vec![1], vec![0x0, 0x04, 0x24]),
            // write default configuration
            SpiTransaction::write_vec(vec![0x10 << 1, 0, 0]),
            // configure port
            SpiTransaction::write_vec(vec![0x25 << 1, 113, 192]),
            // read value on port
            SpiTransaction::transfer(vec![0x45 << 1 | 1], vec![0x0, 0x1, 0x1]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let max = Max11300::try_new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();

        let max = MAX2.init(Mutex::new(max));
        let ports = Ports::new(max);

        let port5 = ports
            .port5
            .into_configured_port(ConfigMode7(
                AVR::InternalRef,
                ADCRANGE::Rg0_10v,
                NSAMPLES::Samples64,
            ))
            .await
            .unwrap();
        let val = port5.get_value().await.unwrap();
        assert_eq!(val, 257);
        pin.done();
        spi.done();
    }

    #[tokio::test]
    async fn multiport() {
        let config = DeviceConfig::default();
        let pin_expectations = [
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
            PinTransaction::set(PinState::Low),
            PinTransaction::set(PinState::High),
        ];
        let spi_expectations = [
            // connection check
            SpiTransaction::transfer(vec![1], vec![0x0, 0x04, 0x24]),
            // write default configuration
            SpiTransaction::write_vec(vec![0x10 << 1, 0, 0]),
            // configure ports
            SpiTransaction::write_vec(vec![0x25 << 1, 81, 0]),
            SpiTransaction::write_vec(vec![0x26 << 1, 81, 0]),
            // set value on port
            SpiTransaction::write_vec(vec![0x65 << 1, 0, 42, 0, 43]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let max = Max11300::try_new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();

        let max = MAX3.init(Mutex::new(max));
        let ports = Ports::new(max);

        let port5 = ports
            .port5
            .into_configured_port(ConfigMode5(DACRANGE::Rg0_10v))
            .await
            .unwrap();
        let port6 = ports
            .port6
            .into_configured_port(ConfigMode5(DACRANGE::Rg0_10v))
            .await
            .unwrap();
        let mut mp = Multiport::new([port5, port6]).unwrap();
        mp.set_values(&[42, 43]).await.unwrap();
        pin.done();
        spi.done();
    }
}
