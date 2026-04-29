#![no_std]
#![no_main]
use crate::doubloon::ahrs::ahrs;
use crate::doubloon::compass::{read_accelerometer, read_magnetometer};
use crate::doubloon::gyro::read_gyro;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, blocking_mutex::raw::NoopRawMutex, mutex::Mutex,
    signal::Signal,
};
use i3g4250d::I3G4250D;
use nalgebra::Vector3;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use static_cell::StaticCell;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};
mod doubloon;

enum CalibrationState {
    Start,
    Stop,
}

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
});

type SharedI2CBusMutex = Mutex<NoopRawMutex, I2c<'static, Async, i2c::Master>>;
type Gyro = I3G4250D<Spi<'static, Blocking, spi::mode::Master>, Output<'static>>;
pub type GyroMutex = Mutex<CriticalSectionRawMutex, Gyro>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let ps = embassy_stm32::init(Default::default());
    let i2c = I2c::new(
        ps.I2C1,
        ps.PB6,
        ps.PB7,
        ps.DMA1_CH6,
        ps.DMA1_CH7,
        Irqs,
        Default::default(),
    );

    static MAG_MEASUREMENT: Signal<CriticalSectionRawMutex, Vector3<f32>> = Signal::new();
    static GYRO_MEASUREMENT: Signal<CriticalSectionRawMutex, Vector3<f32>> = Signal::new();
    static ACCEL_MEASUREMENT: Signal<CriticalSectionRawMutex, Vector3<f32>> = Signal::new();
    static MAGCAL: Signal<CriticalSectionRawMutex, CalibrationState> = Signal::new();
    static I2C_CELL: StaticCell<SharedI2CBusMutex> = StaticCell::new();
    let i2c_bus = I2C_CELL.init(Mutex::new(i2c));
    spawner.spawn(read_magnetometer(i2c_bus, 10, &MAGCAL, &MAG_MEASUREMENT).unwrap());
    spawner.spawn(read_accelerometer(i2c_bus, 10, &ACCEL_MEASUREMENT).unwrap());

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(1_000_000);
    let spi = Spi::new_blocking(ps.SPI1, ps.PA5, ps.PA7, ps.PA6, spi_config);
    let cs_pin = Output::new(ps.PE3, Level::High, Speed::Low);

    if let Some(i3g4250d) = I3G4250D::new(spi, cs_pin).ok() {
        static I3G4250D_CELL: StaticCell<GyroMutex> = StaticCell::new();
        let i3g4250d = I3G4250D_CELL.init(Mutex::new(i3g4250d));
        spawner.spawn(read_gyro(i3g4250d, 10, &MAGCAL, &GYRO_MEASUREMENT).unwrap());
    } else {
        defmt::error!("Could not establish SPI connection to i3g4250 gyro.");
    }

    spawner.spawn(ahrs(&MAG_MEASUREMENT, &GYRO_MEASUREMENT, &ACCEL_MEASUREMENT).unwrap());
}
