#![no_std]

use core::cell::RefCell;

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use heapless::Vec;
use seq_macro::seq;

/// Static device id
const DEVICE_ID: u16 = 0x0424;
/// Highest addressable register address
const MAX_ADDRESS: u8 = 0x73;

/// Device ID Register
const REG_DEVICE_ID: u8 = 0x00;
/// Device Control Register
const REG_DEVICE_CTRL: u8 = 0x10;
/// First port config register
const REG_PORT_CONFIG: u8 = 0x20;
/// First ADC data register
const REG_ADC_DATA: u8 = 0x40;
/// First DAC data register
const REG_DAC_DATA: u8 = 0x60;

#[derive(Debug)]
pub enum Error<S, P> {
    /// SPI bus error
    Spi(S),
    /// Pin error
    Pin(P),
    /// Connection error (device not found)
    Conn,
    /// Address error (invalid or out of bounds)
    Address,
    /// Port error (invalid or out of bounds)
    Port,
}

/// Available Ports
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Port {
    /// Port 0
    P0,
    /// Port 1
    P1,
    /// Port 2
    P2,
    /// Port 3
    P3,
    /// Port 4
    P4,
    /// Port 5
    P5,
    /// Port 6
    P6,
    /// Port 7
    P7,
    /// Port 8
    P8,
    /// Port 9
    P9,
    /// Port 10
    P10,
    /// Port 11
    P11,
    /// Port 12
    P12,
    /// Port 13
    P13,
    /// Port 14
    P14,
    /// Port 15
    P15,
    /// Port 16
    P16,
    /// Port 17
    P17,
    /// Port 18
    P18,
    /// Port 19
    P19,
}

impl Port {
    fn as_config_addr(&self) -> u8 {
        REG_PORT_CONFIG + *self as u8
    }
    pub fn as_u16(&self) -> u16 {
        *self as u16
    }
}

/// ADC conversion mode selection
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ADCCTL {
    /// Idle mode – The ADC does not perform any conversion (default)
    IdleMode,
    /// Single sweep – The ADC performs one conversion for each of the ADC-configured ports sequentially. The assertion of CNVT triggers the single sweep. The sweep starts with the ADC-configured port of lowest index and stops with the ADC-configured port of highest index.
    SingleSweep,
    /// Single conversion – The ADC performs one conversion for the current port. It starts with the lowest index port that is ADC-configured, and it progresses to higher index ports as CNVT is asserted.
    SingleConversion,
    /// Continuous sweep – This mode is not controlled by CNVT. The ADC continuously sweeps the ADC-configured ports
    ContinuousSweep,
}

/// DAC mode selection
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum DACCTL {
    /// Sequential update mode for DAC-configured ports (default)
    Sequential,
    /// Immediate update mode for DAC-configured ports. The DAC-configured port that received new data is the next port to be updated. After updating that port, the DACconfigured port update sequence continues from that port onward. A minimum of 80µs must be observed before requesting another immediate update
    Immediate,
    /// All DAC-configured ports use the same data stored in DACPRSTDAT1[11:0]
    PresetData1,
    /// All DAC-configured ports use the same data stored in DACPRSTDAT2[11:0]
    PresetData2,
}

/// ADC conversion rate selection
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ADCCONV {
    /// ADC conversion rate of 200ksps (default)
    Rate200,
    /// ADC conversion rate of 250ksps
    Rate250,
    /// ADC conversion rate of 333ksps
    Rate333,
    /// ADC conversion rate of 400ksps
    Rate400,
}

/// DAC voltage reference selection
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum DACREF {
    /// External reference voltage (default)
    ExternalRef,
    /// Internal reference voltage
    InternalRef,
}

/// Thermal shutdown enable
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum THSHDN {
    /// Thermal shutdown function disabled.
    Disabled,
    // Thermal shutdown function enabled. If the internal temperature monitor is enabled, and if the internal temperature is measured to be larger than 145°C, the device is reset, thus bringing all channels to high-impedance mode and setting all registers to their default value.
    Enabled,
}

/// Temperature monitor selection
/// TMPCTL[0]: Internal temperature monitor (0: disabled; 1: enabled)
/// TMPCTL[1]: 1st external temperature monitor (0: disabled; 1: enabled)
/// TMPCTL[2]: 2nd external temperature monitor (0: disabled; 1: enabled)
#[derive(Clone, Copy)]
pub struct TMPCTL(u8, u8, u8);

impl From<TMPCTL> for u16 {
    fn from(tmpctl: TMPCTL) -> Self {
        (tmpctl.0 as u16) << 2 | (tmpctl.1 as u16) << 1 | tmpctl.0 as u16
    }
}

/// Temperature conversion time control
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum TMPPER {
    /// Default conversion time setting. Selected for junction capacitance filter < 100pF.
    Default,
    /// Extended conversion time setting. Selected for junction capacitance filter from 100pF to 390pF
    Extended,
}

/// Temperature sensor series resistor cancellation mode
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum RSCANCEL {
    /// Temperature sensor series resistance cancellation disabled.
    Disabled,
    /// Temperature sensor series resistance cancellation enabled.
    Enabled,
}

/// Power mode selection
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum LPEN {
    /// Default power mode for normal operations
    Default,
    /// Lower power mode. The analog ports are in high-impedance mode. The device can be brought out of the lower power mode by deasserting this bit. The device would then undergo the regular power-on sequence
    LowPower,
}

/// Serial interface burst-mode selection
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum BRST {
    /// Default address incrementing mode. The address is automatically incremented by “1” in burst mode.
    Default,
    /// Contextual address incrementing mode. In burst mode, the address automatically points to the next ADC- or DAC-configured port data register. Specifically, when reading ADC data (writing DAC data), the serial interface reads (writes to) only the data registers of those ports that are ADC-configured (DAC-configured). This mode applies to ADC data read and DAC data write, not DAC data read.
    Contextual,
}

pub struct DeviceConfig {
    pub adcctl: ADCCTL,
    pub dacctl: DACCTL,
    pub adcconv: ADCCONV,
    pub dacref: DACREF,
    pub thshdn: THSHDN,
    pub tmpctl: TMPCTL,
    pub tmpper: TMPPER,
    pub rscancel: RSCANCEL,
    pub lpen: LPEN,
    pub brst: BRST,
}

impl DeviceConfig {
    fn as_u16(&self) -> u16 {
        (self.brst as u16) << 14
            | (self.lpen as u16) << 13
            | (self.rscancel as u16) << 12
            | (self.tmpper as u16) << 11
            | u16::from(self.tmpctl) << 8
            | (self.thshdn as u16) << 7
            | (self.dacref as u16) << 6
            | (self.adcconv as u16) << 4
            | (self.dacctl as u16) << 2
            | self.adcctl as u16
    }
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            adcctl: ADCCTL::IdleMode,
            dacctl: DACCTL::Sequential,
            adcconv: ADCCONV::Rate200,
            dacref: DACREF::ExternalRef,
            thshdn: THSHDN::Disabled,
            tmpctl: TMPCTL(0, 0, 0),
            tmpper: TMPPER::Default,
            rscancel: RSCANCEL::Disabled,
            lpen: LPEN::Default,
            brst: BRST::Default,
        }
    }
}

/// INV (for GPI-controlled functional modes only). Asserted to invert the data received by the GPI-configured port.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum INV {
    /// Data received from GPI-configured port is not inverted
    NotInverted,
    /// Data received from GPI-configured port is inverted
    Inverted,
}

impl INV {
    pub fn mask() -> u16 {
        0xf7ff
    }
    pub fn as_u16(&self) -> u16 {
        (*self as u16) << 11
    }
}

/// AVR (for ADC-related functional modes only). ADC voltage reference selection.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum AVR {
    /// ADC internal voltage reference
    InternalRef,
    /// ADC external voltage reference (all modes except mode 6) or DAC voltage reference determined by DACREF (mode 6 only)
    ExternalRef,
}

impl AVR {
    pub fn mask() -> u16 {
        0xf7ff
    }
    pub fn as_u16(&self) -> u16 {
        (*self as u16) << 11
    }
}

/// ADC Voltage Range. Determines the input voltage range of ports configured in input modes, or the output voltage range of ports configured in output modes.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum ADCRANGE {
    /// 0 to +10v
    Rg0_10v = 1,
    /// -5 to +5v
    RgNeg5_5v,
    /// -10 to 0v
    RgNeg10_0v,
    /// 0 to 2.5v
    Rg0_2v5,
    /// 0 to 2.5v
    Rg0_2v5_1 = 6,
}

impl ADCRANGE {
    pub fn mask() -> u16 {
        0xf8ff
    }
    pub fn as_u16(&self) -> u16 {
        (*self as u16) << 8
    }
}

/// DAC Voltage Range. Determines the input voltage range of ports configured in input modes, or the output voltage range of ports configured in output modes.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum DACRANGE {
    /// 0 to +10v
    Rg0_10v = 1,
    /// -5 to +5v
    RgNeg5_5v,
    /// -10 to 0v
    RgNeg10_0v,
    /// -5 to +5v
    RgNeg5_5v1,
    /// 0 to +10v
    Rg0_10v1 = 6,
}

impl DACRANGE {
    pub fn mask() -> u16 {
        0xf8ff
    }
    pub fn as_u16(&self) -> u16 {
        (*self as u16) << 8
    }
}

/// # Of Samples (for ADC-related functional modes only). Defines the number of samples to be captured and averaged before loading the result in the port’s ADC data register. The coding of the number of samples is 2# OF SAMPLES. The number of samples to average can be 1, 2, 4, 8, 16, 32, 64, or 128.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum NSAMPLES {
    /// Just take one sample
    Samples1,
    /// Average over 2 samples
    Samples2,
    /// Average over 4 samples
    Samples4,
    /// Average over 8 samples
    Samples8,
    /// Average over 16 samples
    Samples16,
    /// Average over 32 samples
    Samples32,
    /// Average over 64 samples
    Samples64,
    /// Average over 128 samples
    Samples128,
}

impl NSAMPLES {
    pub fn mask() -> u16 {
        0xff1f
    }
    pub fn as_u16(&self) -> u16 {
        (*self as u16) << 5
    }
}

struct SPIBus<SPI, EN> {
    spi: SPI,
    enable: EN,
}

impl<SPI, EN, S, P> SPIBus<SPI, EN>
where
    SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
    EN: OutputPin<Error = P>,
{
    fn init(spi: SPI, mut enable: EN) -> Result<Self, Error<S, P>> {
        enable.set_high().map_err(Error::Pin)?;
        let mut bus = Self { spi, enable };
        if bus.read_register(REG_DEVICE_ID)? != DEVICE_ID {
            return Err(Error::Conn);
        }
        Ok(bus)
    }

    fn read_register(&mut self, address: u8) -> Result<u16, Error<S, P>> {
        if address > MAX_ADDRESS {
            return Err(Error::Address);
        }
        self.enable.set_low().map_err(Error::Pin)?;
        let mut buf = [(address << 1 | 1), 0, 0];
        self.spi.transfer(&mut buf).map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok((buf[1] as u16) << 8 | buf[2] as u16)
    }

    fn read_registers<'a>(
        &mut self,
        start_address: u8,
        data: &'a mut [u16],
    ) -> Result<&'a [u16], Error<S, P>> {
        if data.len() > 20 || start_address + data.len() as u8 > MAX_ADDRESS {
            return Err(Error::Address);
        }
        // 1 address byte, 2x20 data bytes maximum
        let mut buf: Vec<u8, 41> = Vec::new();
        // Actual size of u16 output buffer times two plus address byte
        buf.resize(data.len() * 2 + 1, 0).ok();
        // Read instruction
        buf[0] = start_address << 1 | 1;
        // Read values into buf
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi.transfer(&mut buf).map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        // Copy to data buffer
        for (i, bytes) in buf[1..].chunks(2).enumerate() {
            data[i] = (bytes[0] as u16) << 8 | bytes[1] as u16;
        }
        Ok(data)
    }

    fn write_register(&mut self, address: u8, data: u16) -> Result<(), Error<S, P>> {
        if address > MAX_ADDRESS {
            return Err(Error::Address);
        }
        self.enable.set_low().map_err(Error::Pin)?;
        self.spi
            .write(&[address << 1, (data >> 8) as u8, (data & 0xff) as u8])
            .map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok(())
    }

    fn write_registers(&mut self, start_address: u8, data: &[u16]) -> Result<(), Error<S, P>> {
        if data.len() > 20 || start_address + data.len() as u8 > MAX_ADDRESS {
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
        self.spi.write(&buf).map_err(Error::Spi)?;
        self.enable.set_high().map_err(Error::Pin)?;
        Ok(())
    }
}

/// High impedance Mode
pub struct ConfigMode0;
impl ConfigMode0 {
    fn as_u16(&self) -> u16 {
        0x0
    }
}

/// Digital input with programmable threshold, GPI
pub struct ConfigMode1;
impl ConfigMode1 {
    fn as_u16(&self) -> u16 {
        0x1 << 12
    }
}

/// Bidirectional level translator terminal
pub struct ConfigMode2;
impl ConfigMode2 {
    fn as_u16(&self) -> u16 {
        0x2 << 12
    }
}

/// Register-driven digital output with DACcontrolled level, GPO
pub struct ConfigMode3;
impl ConfigMode3 {
    fn as_u16(&self) -> u16 {
        0x3 << 12
    }
}

/// Unidirectional path output with DACcontrolled level, GPO
pub struct ConfigMode4(pub INV, pub Port);
impl ConfigMode4 {
    fn as_u16(&self) -> u16 {
        0x4 << 12 | self.0.as_u16() | self.1 as u16
    }
}

/// Analog output for DAC
#[derive(Clone, Copy)]
pub struct ConfigMode5(pub DACRANGE);
impl ConfigMode5 {
    fn as_u16(&self) -> u16 {
        0x5 << 12 | self.0.as_u16()
    }
}

/// Analog output for DAC with ADC monitoring
pub struct ConfigMode6(pub AVR, pub DACRANGE);
impl ConfigMode6 {
    fn as_u16(&self) -> u16 {
        0x6 << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Positive analog input to single-ended ADC
pub struct ConfigMode7(pub AVR, pub ADCRANGE, pub NSAMPLES);
impl ConfigMode7 {
    fn as_u16(&self) -> u16 {
        0x7 << 12 | self.0.as_u16() | self.1.as_u16() | self.2.as_u16()
    }
}

/// Positive analog input to differential ADC
pub struct ConfigMode8(pub AVR, pub ADCRANGE, pub NSAMPLES, pub Port);
impl ConfigMode8 {
    fn as_u16(&self) -> u16 {
        0x8 << 12 | self.0.as_u16() | self.1.as_u16() | self.2.as_u16() | self.3.as_u16()
    }
}

/// Negative analog input to differential ADC
pub struct ConfigMode9(pub AVR, pub ADCRANGE);
impl ConfigMode9 {
    fn as_u16(&self) -> u16 {
        0x9 << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Analog output for DAC and negative analog input to differential ADC (pseudo-differential mode)
pub struct ConfigMode10(pub AVR, pub DACRANGE);
impl ConfigMode10 {
    fn as_u16(&self) -> u16 {
        0xa << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Terminal to GPIcontrolled analog switch
pub struct ConfigMode11(pub INV, pub Port);
impl ConfigMode11 {
    fn as_u16(&self) -> u16 {
        0xb << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Terminal to registercontrolled analog switch
pub struct ConfigMode12(pub INV, pub Port);
impl ConfigMode12 {
    fn as_u16(&self) -> u16 {
        0xc << 12
    }
}

seq!(N in 0..=12 {
    pub struct PortMode~N<'a, SPI, EN> {
        config: ConfigMode~N,
        port: Port,
        bus: &'a RefCell<SPIBus<SPI, EN>>,
    }
});

impl<'a, SPI, EN, S, P> PortMode5<'a, SPI, EN>
where
    SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
    EN: OutputPin<Error = P>,
{
    pub fn set_value(&self, data: u16) -> Result<(), Error<S, P>> {
        self.bus
            .borrow_mut()
            .write_register(REG_DAC_DATA + (self.port as u8), data)
    }

    pub fn configure_range(&self, range: DACRANGE) -> Result<(), Error<S, P>> {
        let data = self.config.as_u16() & DACRANGE::mask() | range.as_u16();
        self.bus
            .borrow_mut()
            .write_register(self.port.as_config_addr(), data)
    }
}

impl<'a, SPI, EN, S, P> PortMode7<'a, SPI, EN>
where
    SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
    EN: OutputPin<Error = P>,
{
    pub fn get_value(&self) -> Result<u16, Error<S, P>> {
        self.bus
            .borrow_mut()
            .read_register(REG_ADC_DATA + (self.port as u8))
    }

    pub fn configure_avr(&mut self, avr: AVR) -> Result<(), Error<S, P>> {
        let data = self.config.as_u16() & AVR::mask() | avr.as_u16();
        self.bus
            .borrow_mut()
            .write_register(self.port.as_config_addr(), data)?;
        self.config.0 = avr;
        Ok(())
    }

    pub fn configure_range(&mut self, range: ADCRANGE) -> Result<(), Error<S, P>> {
        let data = self.config.as_u16() & ADCRANGE::mask() | range.as_u16();
        self.bus
            .borrow_mut()
            .write_register(self.port.as_config_addr(), data)?;
        self.config.1 = range;
        Ok(())
    }

    pub fn configure_nsamples(&mut self, nsamples: NSAMPLES) -> Result<(), Error<S, P>> {
        let data = self.config.as_u16() & NSAMPLES::mask() | nsamples.as_u16();
        self.bus
            .borrow_mut()
            .write_register(self.port.as_config_addr(), data)?;
        self.config.2 = nsamples;
        Ok(())
    }
}

pub struct MultiportMode5<'a, SPI, EN, const N: usize> {
    configs: [ConfigMode5; N],
    ports: [Port; N],
    bus: &'a RefCell<SPIBus<SPI, EN>>,
}

impl<'a, SPI, EN, S, P, const N: usize> MultiportMode5<'a, SPI, EN, N>
where
    SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
    EN: OutputPin<Error = P>,
{
    pub fn set_values(&self, data: [u16; N]) -> Result<(), Error<S, P>> {
        self.bus
            .borrow_mut()
            .write_registers(REG_DAC_DATA + (self.ports[0] as u8), &data)
    }

    pub fn configure_range(&mut self, port_no: usize, range: DACRANGE) -> Result<(), Error<S, P>> {
        if port_no > N - 1 {
            return Err(Error::Port);
        }
        let data = self.configs[port_no].as_u16() & DACRANGE::mask() | range.as_u16();
        self.bus
            .borrow_mut()
            .write_register(self.ports[port_no].as_config_addr(), data)?;
        self.configs[port_no].0 = range;
        Ok(())
    }
}

// TODO: Add MultiportMode7

pub trait ConfigurePort<'a, MODE, CONFIG, S, P> {
    fn configure_port(&'a mut self, port: Port, config: CONFIG) -> Result<MODE, Error<S, P>>;
}

pub trait ConfigureMultiport<'a, MODE, CONFIG, S, P, const N: usize> {
    fn configure_multiport(
        &'a mut self,
        ports: [Port; N],
        config: CONFIG,
    ) -> Result<MODE, Error<S, P>>;
}

pub struct MAX11300<SPI, EN> {
    bus: RefCell<SPIBus<SPI, EN>>,
    pub config: DeviceConfig,
}

impl<SPI, EN, S, P> MAX11300<SPI, EN>
where
    SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
    EN: OutputPin<Error = P>,
{
    pub fn init(spi: SPI, enable: EN, config: DeviceConfig) -> Result<Self, Error<S, P>> {
        let bus = RefCell::new(SPIBus::init(spi, enable)?);
        bus.borrow_mut()
            .write_register(REG_DEVICE_CTRL, config.as_u16())?;

        Ok(Self { bus, config })
    }

    pub fn reset(&mut self) -> Result<(), Error<S, P>> {
        self.bus
            .borrow_mut()
            .write_register(REG_DEVICE_CTRL, 1 << 15)
    }
}

seq!(N in 0..=12 {
    impl<'a, SPI, EN, S, P> ConfigurePort<'a, PortMode~N<'a, SPI, EN>, ConfigMode~N, S, P>
        for MAX11300<SPI, EN>
    where
        SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
        EN: OutputPin<Error = P>,
    {
        fn configure_port(
            &'a mut self,
            port: Port,
            config: ConfigMode~N,
        ) -> Result<PortMode~N<'a, SPI, EN>, Error<S, P>> {
            self.bus
                .borrow_mut()
                .write_register(port.as_config_addr(), config.as_u16())?;
            Ok(PortMode~N {
                config,
                port,
                bus: &self.bus,
            })
        }
    }
});

impl<'a, SPI, EN, S, P, const N: usize>
    ConfigureMultiport<'a, MultiportMode5<'a, SPI, EN, N>, ConfigMode5, S, P, N>
    for MAX11300<SPI, EN>
where
    SPI: Transfer<u8, Error = S> + Write<u8, Error = S>,
    EN: OutputPin<Error = P>,
{
    fn configure_multiport(
        &'a mut self,
        ports: [Port; N],
        config: ConfigMode5,
    ) -> Result<MultiportMode5<'a, SPI, EN, N>, Error<S, P>> {
        // Check if all the ports are in a row
        // We might weaken this requirement in the future and use the context based burst mode
        for neighbours in ports.windows(2) {
            if neighbours[1] as u8 != (neighbours[0] as u8) + 1 {
                return Err(Error::Port);
            }
        }
        let data = [config.as_u16(); N];
        // Use the same config for all, initially
        let configs = [config; N];
        self.bus
            .borrow_mut()
            .write_registers(ports[0].as_config_addr(), &data)?;
        Ok(MultiportMode5 {
            configs,
            ports,
            bus: &self.bus,
        })
    }
}
