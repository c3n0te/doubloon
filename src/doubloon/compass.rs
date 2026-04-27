use crate::{CalibrationState, SharedI2CBusMutex};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::{i2c, i2c::I2c, mode::Async};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::Delay;
use embassy_time::Timer;
use lsm303agr::Error as Lsm303agrError;
use lsm303agr::{
    AccelMode, AccelOutputDataRate, Lsm303agr, interface::I2cInterface, mode::MagOneShot,
};

type Magnetometer = Lsm303agr<
    I2cInterface<I2cDevice<'static, NoopRawMutex, I2c<'static, Async, i2c::Master>>>,
    MagOneShot,
>;

fn get_lsm303agr_error_text<E>(err: &Lsm303agrError<E>) -> &'static str {
    match err {
        Lsm303agrError::Comm(_) => "I2C communication error.",
        Lsm303agrError::InvalidInputData => "Invalid input data.",
    }
}

#[allow(dead_code)]
async fn read_temperature(mag_driver: &mut Magnetometer) {
    match mag_driver.temperature().await {
        Ok(temperature) => {
            defmt::info!(
                "Current mag temperature is: {} °C",
                temperature.degrees_celsius()
            );
        }
        Err(err) => {
            defmt::error!(
                "defmt::error reading mag temperature: {}",
                get_lsm303agr_error_text(&err)
            );
        }
    }
}

#[embassy_executor::task]
pub async fn read_mag_temperature_every_n_seconds(
    i2c_bus: &'static SharedI2CBusMutex,
    n_seconds: u64,
) {
    let shared_i2c_device = I2cDevice::new(i2c_bus);
    let mut lsm303agr = Lsm303agr::new_with_i2c(shared_i2c_device);

    loop {
        Timer::after_secs(n_seconds).await;
        read_temperature(&mut lsm303agr).await;
    }
}

async fn read_magnetometer(lsm303agr: &mut Magnetometer) {
    match lsm303agr.magnetic_field().await {
        Ok(mag) => {
            defmt::info!(
                "Magnetometer: x = {} µT, y = {} µT, z = {} µT",
                mag.x_nt() as f32 / 1000.0,
                mag.y_nt() as f32 / 1000.0,
                mag.z_nt() as f32 / 1000.0
            );
        }
        Err(err) => {
            defmt::error!(
                "defmt::error reading magnetometer: {}",
                get_lsm303agr_error_text(&err)
            );
        }
    }
}

async fn calibrate_magnetometer() {
    defmt::info!("Beginning Mag Calibration");
}

#[embassy_executor::task]
pub async fn read_magnetometer_every_n_milliseconds(
    i2c_bus: &'static SharedI2CBusMutex,
    n_millis: u64,
    signal: &'static Signal<CriticalSectionRawMutex, CalibrationState>,
) {
    let shared_i2c_device = I2cDevice::new(i2c_bus);
    let mut lsm303agr = Lsm303agr::new_with_i2c(shared_i2c_device);
    if lsm303agr.init().await.is_err() {
        defmt::error!("Error initializing magnetometer.");
    }

    let cal_start = signal.wait().await;
    match cal_start {
        CalibrationState::Start => {
            calibrate_magnetometer().await;
            loop {
                Timer::after_millis(n_millis).await;
                read_magnetometer(&mut lsm303agr).await;
            }
        }
        CalibrationState::Stop => defmt::error!("Error signaling calibration start"),
    }
}

async fn read_accelerometer(lsm303agr: &mut Magnetometer) {
    match lsm303agr.acceleration().await {
        Ok(accel) => {
            defmt::info!(
                "Accelerometer: x = {} g, y = {} g, z = {} g",
                accel.x_mg() as f32 / 1000.0,
                accel.y_mg() as f32 / 1000.0,
                accel.z_mg() as f32 / 1000.0,
            );
        }
        Err(err) => {
            defmt::error!(
                "defmt::error reading accelerometer: {}",
                get_lsm303agr_error_text(&err)
            );
        }
    }
}

#[embassy_executor::task]
pub async fn read_accelerometer_every_n_milliseconds(
    i2c_bus: &'static SharedI2CBusMutex,
    n_millis: u64,
) {
    let shared_i2c_device = I2cDevice::new(i2c_bus);
    let mut lsm303agr = Lsm303agr::new_with_i2c(shared_i2c_device);
    if lsm303agr.init().await.is_err() {
        defmt::error!("Error initializing accelerometer.");
    }

    if lsm303agr
        .set_accel_mode_and_odr(&mut Delay, AccelMode::Normal, AccelOutputDataRate::Hz100)
        .await
        .is_err()
    {
        defmt::error!("Error setting accelerometer params.");
    }

    loop {
        Timer::after_millis(n_millis).await;
        read_accelerometer(&mut lsm303agr).await;
    }
}
