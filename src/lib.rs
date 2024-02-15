#![cfg_attr(not(test), no_std)]

pub mod config;
mod port;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use heapless::Vec;
use seq_macro::seq;

use config::{DeviceConfig, Port, DEVICE_ID, MAX_ADDRESS, REG_DEVICE_CTRL, REG_DEVICE_ID};

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
}

pub type WrappedDriver<SPI, EN> = Mutex<CriticalSectionRawMutex, MaxDriver<SPI, EN>>;

pub struct Max11300<SPI, EN> {
    max: WrappedDriver<SPI, EN>,
    config: DeviceConfig,
}

seq!(N in 0..20 {
    impl<SPI, EN, S, P> Max11300<SPI, EN>
    where
        SPI: SpiBus<Error = S>,
        EN: OutputPin<Error = P>,
    {
        pub async fn new(spi: SPI, enable: EN, config: DeviceConfig) -> Result<Self, Error<S, P>> {
            let max = MaxDriver::try_new(spi, enable, config).await?;
            Ok(Self { max: Mutex::new(max), config })
        }

        pub fn split(&mut self) -> Parts<'_, SPI, EN> {
            Parts {
                #(
                    port~N: Mode0Port::new(Port::P~N, &self.max),
                )*
            }
        }

        pub fn config(&self) -> DeviceConfig {
            self.config
        }

    }
});

pub struct MaxDriver<SPI, EN> {
    enable: EN,
    spi: SPI,
}

impl<SPI, EN, S, P> MaxDriver<SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    async fn try_new(spi: SPI, mut enable: EN, config: DeviceConfig) -> Result<Self, Error<S, P>> {
        enable.set_high().map_err(Error::Pin)?;
        let mut max = Self { enable, spi };
        if max.read_register(REG_DEVICE_ID).await? != DEVICE_ID {
            return Err(Error::Conn);
        }
        max.write_register(REG_DEVICE_CTRL, config.as_u16()).await?;
        Ok(max)
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
}

seq!(N in 0..20 {
    pub struct Parts<'a, SPI, EN>
    where
        SPI: SpiBus + 'a,
        EN: OutputPin,
    {
        #(
            pub port~N: Mode0Port<'a, SPI, EN>,
        )*
    }
});

#[cfg(test)]
mod tests {
    use crate::{
        config::{ConfigMode1, ConfigMode5, ConfigMode7, ADCRANGE, AVR, DACRANGE, NSAMPLES},
        port::{IntoConfiguredPort, IntoMode},
    };

    use super::*;
    use embedded_hal_mock::eh1::pin::{
        Mock as PinMock, State as PinState, Transaction as PinTransaction,
    };
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

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
        let _max = Max11300::new(spi.clone(), pin.clone(), config)
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
        let mut max = Max11300::new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();
        let ports = max.split();
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
        let mut max = Max11300::new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();
        let ports = max.split();
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
        let mut max = Max11300::new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();
        let ports = max.split();
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
        let mut max = Max11300::new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();
        let ports = max.split();
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
