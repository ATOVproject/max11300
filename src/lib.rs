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

/// Max113xx driver
pub struct Max113xx<SPI, EN> {
    channel_config: [Mode; NUM_PORTS],
    config: DeviceConfig,
    enable: EN,
    spi: SPI,
}

impl<SPI, EN, S, P> Max113xx<SPI, EN>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
{
    /// Create a new Max113xx instance
    pub async fn try_new(
        spi: SPI,
        mut enable: EN,
        config: DeviceConfig,
    ) -> Result<Self, Error<S, P>> {
        enable.set_high().map_err(Error::Pin)?;
        let mut max = Self {
            channel_config: [Mode::Mode0(ConfigMode0); NUM_PORTS],
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
        let Mode::Mode1(_) = self.get_mode(port) else {
            return Err(Error::Mode);
        };
        self._gpi_configure_threshold(port, threshold, mode).await
    }

    /// Configure the digital output level for a ConfigMode3 Port
    pub async fn gpo_configure_level(&mut self, port: Port, level: u16) -> Result<(), Error<S, P>> {
        let Mode::Mode3(_) = self.get_mode(port) else {
            return Err(Error::Mode);
        };
        self._gpo_configure_level(port, level).await
    }

    /// Set a digital output high on a ConfigMode3 Port
    pub async fn gpo_set_high(&mut self, port: Port) -> Result<(), Error<S, P>> {
        let Mode::Mode3(_) = self.get_mode(port) else {
            return Err(Error::Mode);
        };
        self._gpo_set_high(port).await
    }

    /// Set a digital output low on a ConfigMode3 Port
    pub async fn gpo_set_low(&mut self, port: Port) -> Result<(), Error<S, P>> {
        let Mode::Mode3(_) = self.get_mode(port) else {
            return Err(Error::Mode);
        };
        self._gpo_set_low(port).await
    }

    /// Toggle digital output on a ConfigMode3 Port
    pub async fn gpo_toggle(&mut self, port: Port) -> Result<(), Error<S, P>> {
        let Mode::Mode3(_) = self.get_mode(port) else {
            return Err(Error::Mode);
        };
        self._gpo_toggle(port).await
    }

    /// Set a DAC value for a ConfigMode5 Port
    pub async fn dac_set_value(&mut self, port: Port, data: u16) -> Result<(), Error<S, P>> {
        // Setting the DAC register values should be allowed in any mode as it is required
        // for some mode transitions (e.g. -> Mode3)
        self._dac_set_value(port, data).await
    }

    /// Read an ADC value from a ConfigMode7 Port
    pub async fn adc_get_value(&mut self, port: Port) -> Result<u16, Error<S, P>> {
        let Mode::Mode7(_) = self.get_mode(port) else {
            return Err(Error::Mode);
        };
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
        // Read 2 registers (0x06 and 0x07)
        // Works for both 11300 and 11311 as they share the same base address
        // 11311 just uses fewer bits in the result.
        let mut data = [0_u16; 2];
        self.read_registers(REG_GPI_STATUS, &mut data).await?;
        Ok(((data[0] as u32) << 16) | data[1] as u32)
    }

    /// Get the Mode for a Port
    pub fn get_mode(&self, port: Port) -> Mode {
        self.channel_config[port.as_usize()]
    }

    async fn read_register(&mut self, address: u8) -> Result<u16, Error<S, P>> {
        if address > MAX_ADDRESS {
            return Err(Error::Address);
        }
        let mut buf = [0, 0, 0];
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi
            .transfer(&mut buf, &[(address << 1) | 1])
            .await
            .map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok(((buf[1] as u16) << 8) | buf[2] as u16)
    }

    async fn read_registers<'a>(
        &mut self,
        start_address: u8,
        data: &'a mut [u16],
    ) -> Result<&'a [u16], Error<S, P>> {
        if data.len() > NUM_PORTS || start_address + data.len() as u8 - 1 > MAX_ADDRESS {
            return Err(Error::Address);
        }
        let mut buf: Vec<u8, 41> = Vec::new();
        buf.resize(1 + data.len() * 2, 0).ok();
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi
            .transfer(&mut buf, &[(start_address << 1) | 1])
            .await
            .map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        for (i, bytes) in buf[1..].chunks_exact(2).enumerate() {
            data[i] = ((bytes[0] as u16) << 8) | bytes[1] as u16;
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
        if data.len() > NUM_PORTS || start_address + data.len() as u8 - 1 > MAX_ADDRESS {
            return Err(Error::Address);
        }
        let mut buf: Vec<u8, 41> = Vec::new();
        buf.resize(data.len() * 2 + 1, 0).ok();
        buf[0] = start_address << 1;

        let payload = &mut buf[1..];
        for (bytes, &word) in payload.chunks_exact_mut(2).zip(data.iter()) {
            let word_bytes = word.to_be_bytes();
            bytes[0] = word_bytes[0];
            bytes[1] = word_bytes[1];
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
        // Calculate IRQ Mode register based on port index and ports per register
        let reg_offset = (port as usize) / PORTS_PER_IRQ_REG;
        let pos = ((port as usize) % PORTS_PER_IRQ_REG) * 2;
        let reg_addr = REG_IRQ_MODE + reg_offset as u8;

        let mut current = self.read_register(reg_addr).await?;
        let mut next = (mode as u16) << pos;
        current &= !(0b11 << pos);
        next |= current;
        self.write_register(reg_addr, next).await?;

        // Set threshold for port (Address is REG_DAC_DATA + port index)
        self._dac_set_value(port, threshold).await
    }

    async fn _gpo_configure_level(&mut self, port: Port, level: u16) -> Result<(), Error<S, P>> {
        self._dac_set_value(port, level).await
    }

    // Helper to calculate GPO register address and bit mask
    fn get_gpo_reg_and_mask(port: Port) -> (u8, u16) {
        let p_idx = port as usize;
        if p_idx < PORTS_IN_GPO_REG0 {
            (REG_GPO_DATA, 1 << p_idx)
        } else {
            (REG_GPO_DATA + 1, 1 << (p_idx - PORTS_IN_GPO_REG0))
        }
    }

    async fn _gpo_modify<F>(&mut self, port: Port, f: F) -> Result<(), Error<S, P>>
    where
        F: FnOnce(u16, u16) -> u16,
    {
        let (addr, mask) = Self::get_gpo_reg_and_mask(port);
        let current = self.read_register(addr).await?;
        let next = f(current, mask);
        self.write_register(addr, next).await
    }

    async fn _gpo_set_high(&mut self, port: Port) -> Result<(), Error<S, P>> {
        self._gpo_modify(port, |curr, mask| curr | mask).await
    }

    async fn _gpo_set_low(&mut self, port: Port) -> Result<(), Error<S, P>> {
        self._gpo_modify(port, |curr, mask| curr & !mask).await
    }

    async fn _gpo_toggle(&mut self, port: Port) -> Result<(), Error<S, P>> {
        self._gpo_modify(port, |curr, mask| curr ^ mask).await
    }

    async fn _dac_set_value(&mut self, port: Port, data: u16) -> Result<(), Error<S, P>> {
        self.write_register(REG_DAC_DATA + (port as u8), data).await
    }

    async fn _adc_get_value(&mut self, port: Port) -> Result<u16, Error<S, P>> {
        self.read_register(REG_ADC_DATA + (port as u8)).await
    }
}

seq!(N in 0..=12 {
    impl<SPI, EN, S, P> ConfigurePort<ConfigMode~N, S, P> for Max113xx<SPI, EN>
    where
        SPI: SpiBus<Error = S>,
        EN: OutputPin<Error = P>,
    {
        async fn configure_port(&mut self, port: Port, config: ConfigMode~N) -> Result<(), Error<S, P>> {
            self._configure_port(port, config.as_u16()).await?;
            self.channel_config[port.as_usize()] = Mode::Mode~N(config);
            Ok(())
        }
    }
});

pub struct ReadInterrupts<SPI: 'static, EN: 'static, M: RawMutex + 'static> {
    max: &'static Mutex<M, Max113xx<SPI, EN>>,
}

impl<SPI, EN, M, S, P> ReadInterrupts<SPI, EN, M>
where
    SPI: SpiBus<Error = S>,
    EN: OutputPin<Error = P>,
    M: RawMutex,
{
    pub fn new(max: &'static Mutex<M, Max113xx<SPI, EN>>) -> Self {
        Self { max }
    }

    pub async fn read(&self) -> Result<Interrupts, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.int_read().await
    }

    pub async fn read_raw(&self) -> Result<u16, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.int_read_raw().await
    }

    pub async fn read_gpist(&self) -> Result<u32, Error<S, P>> {
        let mut driver = self.max.lock().await;
        driver.int_read_gpist().await
    }
}

// Macro to generate the Ports struct with correct number of fields based on feature flag
macro_rules! generate_ports_struct {
    ($limit:expr) => {
        seq!(N in 0..$limit {
            pub struct Ports<SPI: 'static, EN: 'static, M: RawMutex + 'static> {
                pub interrupts: ReadInterrupts<SPI, EN, M>,
                #(
                    pub port~N: Mode0Port<SPI, EN, M>,
                )*
            }

            impl<SPI, EN, M> Ports<SPI, EN, M>
            where
                SPI: SpiBus,
                EN: OutputPin,
                M: RawMutex,
            {
                pub fn new(max: &'static Mutex<M, Max113xx<SPI, EN>>) -> Self {
                    Self {
                        interrupts: ReadInterrupts::new(max),
                        #(
                            port~N: Mode0Port::new(Port::P~N, max),
                        )*
                    }
                }
            }
        });
    }
}

#[cfg(feature = "max11300")]
generate_ports_struct!(20);

#[cfg(feature = "max11311")]
generate_ports_struct!(12);

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

    type MockMax = Max113xx<Generic<SpiTransaction<u8>>, Generic<PinTransaction>>;

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
        let _max = Max113xx::try_new(spi.clone(), pin.clone(), config)
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
        let max = Max113xx::try_new(spi.clone(), pin.clone(), config)
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
        let max = Max113xx::try_new(spi.clone(), pin.clone(), config)
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
            SpiTransaction::transfer(vec![(0x45 << 1) | 1], vec![0x0, 0x1, 0x1]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let max = Max113xx::try_new(spi.clone(), pin.clone(), config)
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
    async fn config_mode_7_direct() {
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
            SpiTransaction::transfer(vec![(0x45 << 1) | 1], vec![0x0, 0x1, 0x1]),
        ];
        let mut pin = PinMock::new(&pin_expectations);
        let mut spi = SpiMock::new(&spi_expectations);
        let mut max = Max113xx::try_new(spi.clone(), pin.clone(), config)
            .await
            .unwrap();

        let port = Port::try_from(5).unwrap();
        max.configure_port(
            port,
            ConfigMode7(AVR::InternalRef, ADCRANGE::Rg0_10v, NSAMPLES::Samples64),
        )
        .await
        .unwrap();

        let val = max.adc_get_value(port).await.unwrap();

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
        let max = Max113xx::try_new(spi.clone(), pin.clone(), config)
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
