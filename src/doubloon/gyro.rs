use crate::GyroMutex;
use embassy_stm32::spi::Error as SpiError;
use embassy_time::Timer;
use i3g4250d::I16x3;

const CAL_SAMPLE_SIZE: usize = 1000;

async fn calibrate_gyro(i3g4250d: &'static GyroMutex) -> Result<I16x3, &'static str> {
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

async fn read_gyro(i3g4250d: &'static GyroMutex, cal_offsets: &I16x3) {
    match i3g4250d.lock().await.gyro() {
        Ok(gyro_all) => {
            let calibrated = I16x3 {
                x: gyro_all.x - cal_offsets.x,
                y: gyro_all.y - cal_offsets.y,
                z: gyro_all.z - cal_offsets.z,
            };
            defmt::info!(
                "Gyroscope: x = {} rad/s, y = {} rad/s, z = {} rad/s",
                calibrated.x as f32 * core::f32::consts::PI / 180.0,
                calibrated.y as f32 * core::f32::consts::PI / 180.0,
                calibrated.z as f32 * core::f32::consts::PI / 180.0
            );
        }
        Err(err) => {
            defmt::error!("ERROR reading gyro values: {}", get_spi_error_text(&err));
        }
    }
}

#[embassy_executor::task]
pub async fn read_gyro_every_n_milliseconds(i3g4250d: &'static GyroMutex, n_millis: u64) {
    match calibrate_gyro(i3g4250d).await {
        Ok(cal_offsets) => loop {
            Timer::after_millis(n_millis).await;
            read_gyro(i3g4250d, &cal_offsets).await;
        },
        Err(e) => {
            defmt::error!("{}", e);
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
