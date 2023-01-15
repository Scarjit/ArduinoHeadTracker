use nalgebra::{Matrix3, Vector3};

pub(crate) struct KalmanFilter {
    pub(crate) state: Vector3<f64>,
    pub(crate) cov: Matrix3<f64>,
    pub(crate) transition: Matrix3<f64>,
    pub(crate) control: Matrix3<f64>,
    pub(crate) measurement: Matrix3<f64>,
    pub(crate) process_noise: Matrix3<f64>,
    pub(crate) measurement_noise: Matrix3<f64>,
}

impl KalmanFilter {
    pub(crate) fn predict(&mut self, control: Vector3<f64>) {
        self.state = self.transition.clone() * self.state.clone() + self.control.clone() * control;
        self.cov = self.transition.clone() * self.cov.clone() * self.transition.clone().transpose() + self.process_noise.clone();
    }

    pub(crate) fn update(&mut self, measurement: Vector3<f64>) {
        let innovation = measurement - self.measurement.clone() * self.state.clone();
        let innovation_cov = self.measurement.clone() * self.cov.clone() * self.measurement.clone().transpose() + self.measurement_noise.clone();
        let kalman_gain = self.cov.clone() * self.measurement.clone().transpose() * innovation_cov.try_inverse().unwrap();
        self.state = self.state.clone() + kalman_gain.clone() * innovation;
        self.cov = (Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0) - kalman_gain.clone() * self.measurement.clone()) * self.cov.clone();
    }
}

pub(crate) fn tilt_compensation(angle_x: f64, angle_y: f64, angle_z: f64, acc_x: f64, acc_y: f64, acc_z: f64) -> Vector3<f64> {
    let roll = angle_x;
    let pitch = angle_y;
    let yaw = angle_z;
    let acc = Vector3::new(acc_x, acc_y, acc_z);
    let rot_x = Matrix3::new(
        1.0, 0.0, 0.0,
        0.0, pitch.cos(), -pitch.sin(),
        0.0, pitch.sin(), pitch.cos()
    );
    let rot_y = Matrix3::new(
        roll.cos(), 0.0, roll.sin(),
        0.0, 1.0, 0.0,
        -roll.sin(), 0.0, roll.cos()
    );
    let rot_z = Matrix3::new(
        yaw.cos(), -yaw.sin(), 0.0,
        yaw.sin(), yaw.cos(), 0.0,
        0.0, 0.0, 1.0
    );
    let rotation_matrix = rot_z * rot_y * rot_x;
    rotation_matrix.try_inverse().unwrap() * acc
}
