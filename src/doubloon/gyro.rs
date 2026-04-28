use crate::{CalibrationState, GyroMutex};
use embassy_stm32::spi::Error as SpiError;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use i3g4250d::I16x3;
use nalgebra::Vector3;

async fn calibrate_gyroscope(i3g4250d: &'static GyroMutex) -> Result<I16x3, &'static str> {
    const CAL_SAMPLE_SIZE: u32 = 1000;
    defmt::info!(
        "Calibrating I3G4250d gyroscope with {} samples. ETA is 10 seconds. Hold the gyro flat to the earth's surface.",
        CAL_SAMPLE_SIZE
    );

    let mut average = I16x3 { x: 0, y: 0, z: 0 };
    let mut x_sum = 0;
    let mut y_sum = 0;
    let mut z_sum = 0;

    for _ in 0..CAL_SAMPLE_SIZE {
        match i3g4250d.lock().await.gyro() {
            Ok(gyro_data) => {
                x_sum += gyro_data.x as i32;
                y_sum += gyro_data.y as i32;
                z_sum += gyro_data.z as i32;

                average.x = (x_sum / CAL_SAMPLE_SIZE as i32) as i16;
                average.y = (y_sum / CAL_SAMPLE_SIZE as i32) as i16;
                average.z = (z_sum / CAL_SAMPLE_SIZE as i32) as i16;
            }
            Err(_) => return Err("ERROR while calibrating gyro! Could not read gyro data."),
        }

        Timer::after_millis(10).await;
    }

    Ok(average)
}

fn get_spi_error_text(err: &SpiError) -> &'static str {
    match err {
        SpiError::Framing => "SPI invalid framing",
        SpiError::Crc => "SPI CRC check error. Is CRC even enabled?",
        SpiError::ModeFault => "SPI mode faulty",
        SpiError::Overrun => "SPI overrun",
    }
}

#[embassy_executor::task]
pub async fn read_gyro(
    i3g4250d: &'static GyroMutex,
    n_millis: u64,
    magcal: &'static Signal<CriticalSectionRawMutex, CalibrationState>,
    gyro_meas: &'static Signal<CriticalSectionRawMutex, Vector3<f32>>,
) {
    let Ok(cal_offsets) = calibrate_gyroscope(i3g4250d).await else {
        magcal.signal(CalibrationState::Stop);
        defmt::error!("Error calibrating gyroscope");
        return;
    };

    magcal.signal(CalibrationState::Start);
    loop {
        Timer::after_millis(n_millis).await;
        match i3g4250d.lock().await.gyro() {
            Ok(gyro_all) => {
                let calibrated = I16x3 {
                    x: gyro_all.x - cal_offsets.x,
                    y: gyro_all.y - cal_offsets.y,
                    z: gyro_all.z - cal_offsets.z,
                };

                let gyro = Vector3::new(
                    calibrated.x as f32,
                    calibrated.y as f32,
                    calibrated.z as f32,
                );

                gyro_meas.signal(gyro);
            }
            Err(err) => {
                defmt::error!("ERROR reading gyro values: {}", get_spi_error_text(&err));
            }
        }
    }
}

#[allow(dead_code)]
async fn read_gyro_temperature(i3g4250d: &'static GyroMutex) {
    match i3g4250d.lock().await.temp() {
        Ok(temperature) => {
            defmt::info!("Current gyro temperature is {} °C", temperature);
        }
        Err(err) => {
            defmt::error!(
                "ERROR reading gyro temperature values: {}",
                get_spi_error_text(&err)
            );
        }
    }
}

#[embassy_executor::task]
pub async fn read_gyro_temperature_every_n_seconds(i3g4250d: &'static GyroMutex, n_seconds: u64) {
    loop {
        Timer::after_secs(n_seconds).await;
        read_gyro_temperature(i3g4250d).await;
    }
}
