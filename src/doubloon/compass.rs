use crate::{CalibrationState, SharedI2CBusMutex};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::{i2c, i2c::I2c, mode::Async};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::Delay;
use embassy_time::Timer;
use heapless::Vec;
use lsm303agr::Error as Lsm303agrError;
use lsm303agr::{
    AccelMode, AccelOutputDataRate, Lsm303agr, interface::I2cInterface, mode::MagOneShot,
};
use nalgebra::Vector3;

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
                "Error reading mag temperature: {}",
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

async fn calibrate_magnetometer(lsm303agr: &mut Magnetometer) -> (Vector3<f32>, Vector3<f32>) {
    const NUM_SAMPLES: usize = 1000;
    defmt::info!(
        "Calibrating LSM303AGR Magnetometer with {} samples. Wave sensor in figure eight for ~20 seconds.",
        NUM_SAMPLES,
    );

    let (mut x, mut y, mut z): (
        Vec<f32, NUM_SAMPLES>,
        Vec<f32, NUM_SAMPLES>,
        Vec<f32, NUM_SAMPLES>,
    ) = (Vec::new(), Vec::new(), Vec::new());

    for _ in 0..NUM_SAMPLES {
        match lsm303agr.magnetic_field().await {
            Ok(mag) => {
                if x.push(mag.x_nt() as f32 / 1000.0).is_err() {
                    defmt::error!("Error: failed to push mag_x into vec")
                };

                if y.push(mag.y_nt() as f32 / 1000.0).is_err() {
                    defmt::error!("Error: failed to push mag_y into vec")
                };

                if z.push(mag.z_nt() as f32 / 1000.0).is_err() {
                    defmt::error!("Error: failed to push mag_z into vec")
                };
            }
            Err(err) => {
                defmt::error!(
                    "Error reading magnetometer: {}",
                    get_lsm303agr_error_text(&err)
                );
            }
        }

        Timer::after_millis(10).await;
    }

    let hard_iron_x = (x.iter().max_by(|x, y| x.total_cmp(y)).unwrap()
        + x.iter().min_by(|x, y| x.total_cmp(y)).unwrap())
        / 2.0;

    let hard_iron_y = (y.iter().max_by(|x, y| x.total_cmp(y)).unwrap()
        + y.iter().min_by(|x, y| x.total_cmp(y)).unwrap())
        / 2.0;

    let hard_iron_z = (z.iter().max_by(|x, y| x.total_cmp(y)).unwrap()
        + z.iter().min_by(|x, y| x.total_cmp(y)).unwrap())
        / 2.0;

    let avg_delta_x = (x.iter().max_by(|x, y| x.total_cmp(y)).unwrap()
        - x.iter().min_by(|x, y| x.total_cmp(y)).unwrap())
        / 2.0;

    let avg_delta_y = (y.iter().max_by(|x, y| x.total_cmp(y)).unwrap()
        - y.iter().min_by(|x, y| x.total_cmp(y)).unwrap())
        / 2.0;

    let avg_delta_z = (z.iter().max_by(|x, y| x.total_cmp(y)).unwrap()
        - z.iter().min_by(|x, y| x.total_cmp(y)).unwrap())
        / 2.0;

    let avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;
    let soft_iron_x = avg_delta / avg_delta_x;
    let soft_iron_y = avg_delta / avg_delta_y;
    let soft_iron_z = avg_delta / avg_delta_z;

    let (hard_iron, soft_iron) = (
        Vector3::new(hard_iron_x, hard_iron_y, hard_iron_z),
        Vector3::new(soft_iron_x, soft_iron_y, soft_iron_z),
    );

    (hard_iron, soft_iron)
}

#[embassy_executor::task]
pub async fn read_magnetometer(
    i2c_bus: &'static SharedI2CBusMutex,
    n_millis: u64,
    magcal: &'static Signal<CriticalSectionRawMutex, CalibrationState>,
    mag_meas: &'static Signal<CriticalSectionRawMutex, Vector3<f32>>,
) {
    let shared_i2c_device = I2cDevice::new(i2c_bus);
    let mut lsm303agr = Lsm303agr::new_with_i2c(shared_i2c_device);
    if lsm303agr.init().await.is_err() {
        defmt::error!("Error initializing magnetometer.");
    }

    let cal_start = magcal.wait().await;
    let (mut hard_iron, mut soft_iron): (Vector3<f32>, Vector3<f32>) =
        (Vector3::zeros(), Vector3::zeros());
    match cal_start {
        CalibrationState::Start => {
            (hard_iron, soft_iron) = calibrate_magnetometer(&mut lsm303agr).await;
        }
        CalibrationState::Stop => defmt::error!("Error signaling calibration start"),
    }

    loop {
        Timer::after_millis(n_millis).await;
        match lsm303agr.magnetic_field().await {
            Ok(magnetometer) => {
                let mag = Vector3::new(
                    ((magnetometer.x_nt() as f32 / 1000.0) - hard_iron.x) * soft_iron.x,
                    ((magnetometer.y_nt() as f32 / 1000.0) - hard_iron.y) * soft_iron.y,
                    ((magnetometer.z_nt() as f32 / 1000.0) - hard_iron.z) * soft_iron.z,
                );

                mag_meas.signal(mag);
            }
            Err(err) => {
                defmt::error!(
                    "Error reading magnetometer: {}",
                    get_lsm303agr_error_text(&err)
                );
            }
        }
    }
}

#[embassy_executor::task]
pub async fn read_accelerometer(
    i2c_bus: &'static SharedI2CBusMutex,
    n_millis: u64,
    accel_meas: &'static Signal<CriticalSectionRawMutex, Vector3<f32>>,
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
        match lsm303agr.acceleration().await {
            Ok(accel) => {
                let accel = Vector3::new(
                    accel.x_mg() as f32 / 1000.0,
                    accel.y_mg() as f32 / 1000.0,
                    accel.z_mg() as f32 / 1000.0,
                );

                accel_meas.signal(accel);
            }
            Err(err) => {
                defmt::error!(
                    "Error reading accelerometer: {}",
                    get_lsm303agr_error_text(&err)
                );
            }
        }
    }
}
