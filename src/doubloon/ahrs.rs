use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use fusion_ahrs::Ahrs;
use nalgebra::Vector3;

#[embassy_executor::task]
pub async fn ahrs(
    mag: &'static Signal<CriticalSectionRawMutex, Vector3<f32>>,
    gyro: &'static Signal<CriticalSectionRawMutex, Vector3<f32>>,
    accel: &'static Signal<CriticalSectionRawMutex, Vector3<f32>>,
) {
    const DELTA_TIME: f32 = 0.01;
    let mut ahrs = Ahrs::new();

    loop {
        let magnetometer = mag.wait().await;
        let gyroscope = gyro.wait().await;
        let accelerometer = accel.wait().await;
        ahrs.update(gyroscope, accelerometer, magnetometer, DELTA_TIME);
        let orientation = ahrs.quaternion();
        let (roll, pitch, yaw) = orientation.euler_angles();

        /*
        defmt::info!(
            "Magnetometer: x = {} µT, y = {} µT, z = {} µT",
            magnetometer.x,
            magnetometer.y,
            magnetometer.z,
        );

        defmt::info!(
            "Gyroscope: x = {} °/s, y = {} °/s, z = {} °/s",
            gyroscope.x,
            gyroscope.y,
            gyroscope.z
        );

        defmt::info!(
            "Accelerometer: x = {} g, y = {} g, z = {} g",
            accelerometer.x,
            accelerometer.y,
            accelerometer.z,
        );
        */

        defmt::info!(
            "roll: {}°; pitch: {}°; yaw: {}°",
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees()
        );
    }
}
