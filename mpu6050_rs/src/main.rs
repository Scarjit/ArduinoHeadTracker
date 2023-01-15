use std::io::{Read, Write};
use std::net::UdpSocket;

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::mpsc::{Receiver, Sender};
use std::thread;
use nalgebra::{Matrix3, Vector3};
use serialport::DataBits;

pub(crate) mod kalman_filter;

#[derive(Serialize, Deserialize,Debug)]
struct SensorData {
    data: Data,
    status: String,
}

#[derive(Serialize, Deserialize,Debug)]
struct Data {
    temp: f64,
    accX: f64,
    accY: f64,
    accZ: f64,
    gyroX: f64,
    gyroY: f64,
    gyroZ: f64,
    accAngleX: f64,
    accAngleY: f64,
    angleX: f64,
    angleY: f64,
    angleZ: f64,
}


fn main() {
    // Create a channel to send data from serial_reader to udp_sender
    let (tx, rx): (Sender<String>, Receiver<String>) = std::sync::mpsc::channel();

    thread::spawn(move || {
        serial_reader(tx);
    });

    thread::spawn(move || {
        udp_sender(rx);
    });

    loop {
        thread::sleep(std::time::Duration::from_millis(1000));
    }
}

fn serial_reader(tx: Sender<String>) {
    let mut serial_port = serialport::new("COM3", 115200).data_bits(DataBits::Eight).stop_bits(serialport::StopBits::One).flow_control(serialport::FlowControl::None).open().unwrap();

    let mut data = String::with_capacity(245);
    loop {
        data.clear();
        // Read byte by byte until first newline
        let mut buffer = [0u8; 1];
        loop {
            serial_port
                .read_exact(&mut buffer)
                .expect("Failed to read from port");
            if buffer[0] == b'\n' {
                break;
            }
            data.push(buffer[0] as char);
        }

        // Send data to udp_sender
        tx.send(data.clone())
            .expect("Failed to send data to udp_sender");
    }
}

fn udp_sender(rx: Receiver<String>) {
    let udp_client = UdpSocket::bind("127.0.0.1:4444").expect("Failed to bind socket");
    udp_client
        .connect("127.0.0.1:4242")
        .expect("Failed to connect to server");

    let mut kf = kalman_filter::KalmanFilter {
        state: Vector3::new(0.0, 0.0, 0.0),
        cov: Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        transition: Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        control: Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        measurement: Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        process_noise: Matrix3::new(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1),
        measurement_noise: Matrix3::new(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1),
    };

    let now = std::time::Instant::now();

    loop {
        let line = rx
            .recv()
            .expect("Failed to receive data from serial_reader");

        // Parse data as JSON
        let serial_data: SensorData = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(e) => {
                println!("Error: {e}: {line}");
                continue;
            }
        };

        if serial_data.status != "ready" {
            println!("Status: {}", serial_data.status);
            println!("Data: {:?}", serial_data);
            continue;
        }


        // Read angleX, angle_y, angle_z
        let angle_x = serial_data.data.angleX;
        let angle_y = serial_data.data.angleY;
        let angle_z = serial_data.data.angleZ;

        // Read accX, accY, accZ
        let acc_x = serial_data.data.accX;
        let acc_y = serial_data.data.accY;
        let acc_z = serial_data.data.accZ;

        let control = Vector3::new(0.0, 0.0, 0.0);
        let measurement = Vector3::new(acc_x, acc_y, acc_z);
        kf.predict(control);
        kf.update(measurement);
        let compensated_measurement = kalman_filter::tilt_compensation(angle_x, angle_y, angle_z, acc_x, acc_y, acc_z);

        let comp_x = compensated_measurement.x;
        let comp_y = compensated_measurement.y;
        let comp_z = compensated_measurement.z;
        // Print comp data, with 4 decimal places
        println!("Comp: {:.4}, {:.4}, {:.4}", comp_x, comp_y, comp_z);


        // Allocate 48 bytes for data
        let mut data = [0u8; 48];

        // First 24 bytes are zeros

        // Write angle_z to next 8 bytes
        data[24..32].copy_from_slice(&angle_x.to_le_bytes());

        // Write angle_y to next 8 bytes
        data[32..40].copy_from_slice(&angle_y.to_le_bytes());

        // Write angle_x to next 8 bytes
        data[40..48].copy_from_slice(&angle_z.to_le_bytes());

        // Send data to server
        udp_client
            .send(&data)
            .expect("Failed to send data to server");
    }
}
