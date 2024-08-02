/// Static device id
pub const DEVICE_ID: u16 = 0x0424;
/// Highest addressable register address
pub const MAX_ADDRESS: u8 = 0x73;

/// Device ID Register
pub const REG_DEVICE_ID: u8 = 0x00;
/// Interrupt Register
pub const REG_INTERRUPT: u8 = 0x01;
/// First GPI status register
pub const REG_GPI_STATUS: u8 = 0x06;
/// First GPO data register
pub const REG_GPO_DATA: u8 = 0x0D;
/// Device Control Register
pub const REG_DEVICE_CTRL: u8 = 0x10;
/// Interrupt Mask Register
pub const REG_INTERRUPT_MASK: u8 = 0x11;
/// First IRQ Mode Register
pub const REG_IRQ_MODE: u8 = 0x12;
/// First port config register
pub const REG_PORT_CONFIG: u8 = 0x20;
/// First ADC data register
pub const REG_ADC_DATA: u8 = 0x40;
/// First DAC data register
pub const REG_DAC_DATA: u8 = 0x60;

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
    pub fn as_config_addr(&self) -> u8 {
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
/// * TMPCTL[0]: Internal temperature monitor (0: disabled; 1: enabled)
/// * TMPCTL[1]: 1st external temperature monitor (0: disabled; 1: enabled)
/// * TMPCTL[2]: 2nd external temperature monitor (0: disabled; 1: enabled)
#[derive(Clone, Copy)]
pub struct TMPCTL(u8, u8, u8);

impl From<TMPCTL> for u16 {
    fn from(tmpctl: TMPCTL) -> Self {
        (tmpctl.2 as u16) << 2 | (tmpctl.1 as u16) << 1 | tmpctl.0 as u16
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

/// Device configuration. Contains all the config options for intial setup
#[derive(Clone, Copy)]
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
    pub fn as_u16(&self) -> u16 {
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

pub struct Interrupts {
    /// ADC flag interrupt
    /// * Asserted when the ADC completes a conversion (ADC set in single-conversion mode) or when the ADC completes a sweep (ADC set in single-sweep or continuous-sweep mode).
    /// * No interrupt is generated when the ADC is in idle mode.
    /// * Cleared after the interrupt register is read
    pub adcflag: bool,
    /// ADC data ready interrupt
    /// * Asserted when any ADC data register receives a new data sample. If a port is configured to average 2N samples, it takes 2N sweeps for that port data register to be refreshed and assert ADCDR.
    /// * Data registers are refreshed either at the end of a conversion (ADC set in single-conversion mode) or at the end of a sweep (ADC set in single-sweep or continuous-sweep mode).
    /// * Cleared after the interrupt register is read, and after both ADCST[15:0] and ADCST[19:16] registers are read subsequently.
    pub adcdr: bool,
    /// ADC data missed interrupt
    /// * Asserted when the host missed reading a port’s ADC data register by the time that port’s ADC data register is overwritten by new data.
    /// * Cleared after the interrupt register is read
    pub adcdm: bool,
    /// GPI event ready interrupt
    /// * Asserted when a new event is captured by GPI-configured ports. The type of event is set by the corresponding GPI IRQ mode register. The host can then consult GPIST[15:0] and GPIST[19:16] registers to identify the port that caused the interrupt.
    /// * Cleared after the interrupt register is read, and after both GPIST[15:0] and GPIST[19:16] are read subsequently.
    pub gpidr: bool,
    /// GPI event missed interrupt
    /// * Asserted when the host missed reading the GPI status register by the time that register is overwritten.
    /// * Must be used in conjunction with GPIDR for proper operation.
    /// * Cleared after the interrupt register is read, and after both GPIST[15:0] and GPIST[19:16] are read subsequently.
    pub gpidm: bool,
    /// DAC driver overcurrent interrupt
    /// * Asserted when the DAC driver current exceeds approximately 50mA. The host can then read DACOIST[15:0] and DACOIST[19:16] to identify the port that caused the interrupt.
    /// * Cleared after the interrupt register is read, and after both DACOIST[15:0] and DACOIST[19:16] registers are read subsequently.
    pub dacoi: bool,
    // FIXME: continue here
    /// High-voltage supply monitor interrupt
    /// * Asserted when the high voltage supply (AVDDIO) falls below approximately 4V.
    /// * Cleared after the interrupt register is read
    pub vmon: bool,
}

impl From<u16> for Interrupts {
    fn from(int: u16) -> Self {
        Self {
            adcflag: int & (1 << 0) != 0,
            adcdr: int & (1 << 1) != 0,
            adcdm: int & (1 << 2) != 0,
            gpidr: int & (1 << 3) != 0,
            gpidm: int & (1 << 4) != 0,
            dacoi: int & (1 << 5) != 0,
            vmon: int & (1 << 15) != 0,
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

/// GPI IRQ Mode
/// Each input port is controlled by GPIMD. IRQs can be asserted on the rising edge, falling edge, both edges, or never.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum GPIMD {
    Never,
    PositiveEdge,
    NegativeEdge,
    BothEdges,
}

/// High impedance Mode
#[derive(Clone, Copy)]
pub struct ConfigMode0;
impl ConfigMode0 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x0
    }
}

/// Digital input with programmable threshold, GPI
#[derive(Clone, Copy)]
pub struct ConfigMode1;
impl ConfigMode1 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x1 << 12
    }
}

/// Bidirectional level translator terminal
#[derive(Clone, Copy)]
pub struct ConfigMode2;
impl ConfigMode2 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x2 << 12
    }
}

/// Register-driven digital output with DACcontrolled level, GPO
#[derive(Clone, Copy)]
pub struct ConfigMode3;
impl ConfigMode3 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x3 << 12
    }
}

/// Unidirectional path output with DACcontrolled level, GPO
#[derive(Clone, Copy)]
pub struct ConfigMode4(pub INV, pub Port);
impl ConfigMode4 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x4 << 12 | self.0.as_u16() | self.1 as u16
    }
}

/// Analog output for DAC
#[derive(Clone, Copy)]
pub struct ConfigMode5(pub DACRANGE);
impl ConfigMode5 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x5 << 12 | self.0.as_u16()
    }
}

/// Analog output for DAC with ADC monitoring
#[derive(Clone, Copy)]
pub struct ConfigMode6(pub AVR, pub DACRANGE);
impl ConfigMode6 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x6 << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Positive analog input to single-ended ADC
#[derive(Clone, Copy)]
pub struct ConfigMode7(pub AVR, pub ADCRANGE, pub NSAMPLES);
impl ConfigMode7 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x7 << 12 | self.0.as_u16() | self.1.as_u16() | self.2.as_u16()
    }
}

/// Positive analog input to differential ADC
#[derive(Clone, Copy)]
pub struct ConfigMode8(pub AVR, pub ADCRANGE, pub NSAMPLES, pub Port);
impl ConfigMode8 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x8 << 12 | self.0.as_u16() | self.1.as_u16() | self.2.as_u16() | self.3.as_u16()
    }
}

/// Negative analog input to differential ADC
#[derive(Clone, Copy)]
pub struct ConfigMode9(pub AVR, pub ADCRANGE);
impl ConfigMode9 {
    pub(crate) fn as_u16(&self) -> u16 {
        0x9 << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Analog output for DAC and negative analog input to differential ADC (pseudo-differential mode)
#[derive(Clone, Copy)]
pub struct ConfigMode10(pub AVR, pub DACRANGE);
impl ConfigMode10 {
    pub(crate) fn as_u16(&self) -> u16 {
        0xa << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Terminal to GPIcontrolled analog switch
#[derive(Clone, Copy)]
pub struct ConfigMode11(pub INV, pub Port);
impl ConfigMode11 {
    pub(crate) fn as_u16(&self) -> u16 {
        0xb << 12 | self.0.as_u16() | self.1.as_u16()
    }
}

/// Terminal to registercontrolled analog switch
#[derive(Clone, Copy)]
pub struct ConfigMode12(pub INV, pub Port);
impl ConfigMode12 {
    pub(crate) fn as_u16(&self) -> u16 {
        0xc << 12
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config() {
        let config = DeviceConfig::default();
        assert_eq!(config.as_u16(), 0);
    }
}
