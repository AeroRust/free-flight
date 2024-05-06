// examples/angle.rs

//! Free Flight
//!
//! TODO: Document.

#![no_std]
#![feature(start)]
#![deny(missing_docs)]

extern crate libc; // TODO: Remove.
use core::fmt::Write; // TODO: Remove.

extern crate nalgebra as na;

use ahrs::{Ahrs, Madgwick};
use core::f32;
use free_flight_stabilization::{AngleFullStabilizer, FlightStabilizer, FlightStabilizerConfig};
use na::Vector3;

// ======================================================================
// User-Specified Variables
// ======================================================================

// Set the PID gains for roll, pitch, and yaw.
const KP_ROLL_ANGLE: f32 = 0.2;
const KI_ROLL_ANGLE: f32 = 0.3;
const KD_ROLL_ANGLE: f32 = 0.05;
const KP_PITCH_ANGLE: f32 = 0.2;
const KI_PITCH_ANGLE: f32 = 0.3;
const KD_PITCH_ANGLE: f32 = 0.05;
const KP_YAW_ANGLE: f32 = 0.2;
const KI_YAW_ANGLE: f32 = 0.3;
const KD_YAW_ANGLE: f32 = 0.05;
/* TODO: support multiple stabilizers
const KP_ROLL_RATE: f32 = 0.15;
const KI_ROLL_RATE: f32 = 0.2;
const KD_ROLL_RATE: f32 = 0.0002;
const KP_PITCH_RATE: f32 = 0.15;
const KI_PITCH_RATE: f32 = 0.2;
const KD_PITCH_RATE: f32 = 0.0002;
const KP_YAW_RATE: f32 = 0.3;
const KI_YAW_RATE: f32 = 0.05;
const KD_YAW_RATE: f32 = 0.00015;
*/

// Upper limit for PID integral term to prevent windup.
const I_LIMIT: f32 = 25.0;

// Scale to adjust PID outputs to actuator range.
const PID_OUTPUT_SCALE: f32 = 0.01;

// ======================================================================
// Program Constants
// ======================================================================

const EARTH_MAGNETIC_FIELD_MAGNITUDE: f32 = 50.0;  // in microteslas (µT)
const G: f32 = 9.80665; // m^2/s
const UPDATE_HZ: f32 = 1.0;
const DEGREES_TO_RADIANS: f32 = f32::consts::PI / 180.0;
const RADIANS_TO_DEGREES: f32 = 180.0 / f32::consts::PI;
const INDENT: usize = 4;
const HEADING_W: usize = 14;

// Implement minimal formatting features for output.
struct Stdout;

// TODO: Remove.
impl Write for Stdout {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let mut buffer = [0u8; 1024]; // Buffer to hold the string and null terminator

        // Ensure we don't exceed the buffer size
        if s.len() + 1 > buffer.len() {
            return Err(core::fmt::Error);
        }

        // Copy the string into the buffer and null-terminate it
        buffer[..s.len()].copy_from_slice(s.as_bytes());
        buffer[s.len()] = 0; // Null terminator

        unsafe {
            // Use %s to print the string from the buffer
            libc::printf(b"%s\0".as_ptr() as *const _, buffer.as_ptr() as *const _);
        }
        Ok(())
    }
}

fn print_tuple3(output: &mut impl Write, indent: usize, width: usize, heading: &str, tuple: (f32, f32, f32)) {
    writeln!(
        output,
        "{:<indent$}{:<width$} {:-8.3}, {:-8.3}, {:-8.3}",
        "", heading, tuple.0, tuple.1, tuple.2, indent=indent, width=width
    )
    .ok();
}

fn print_vector3(output: &mut impl Write, indent: usize, width: usize, heading: &str, vector3: Vector3<f32>) {
    print_tuple3(output, indent, width, heading, (vector3.x, vector3.y, vector3.z))
}

fn create_stabilizer() -> AngleFullStabilizer::<f32> {
    let mut config = FlightStabilizerConfig::<f32>::new();
    config.kp_roll = KP_ROLL_ANGLE;
    config.ki_roll = KI_ROLL_ANGLE;
    config.kd_roll = KD_ROLL_ANGLE;
    config.kp_pitch = KP_PITCH_ANGLE;
    config.ki_pitch = KI_PITCH_ANGLE;
    config.kd_pitch = KD_PITCH_ANGLE;
    config.kp_yaw = KP_YAW_ANGLE;
    config.ki_yaw = KI_YAW_ANGLE;
    config.kd_yaw = KD_YAW_ANGLE;
    config.i_limit = I_LIMIT;
    config.scale = PID_OUTPUT_SCALE;

    AngleFullStabilizer::with_config(config)
}

fn create_ahrs() -> Madgwick<f32> {
    let sample_period = 1.0 / UPDATE_HZ;
    let beta = 0.04;
    let axisangle = Vector3::y(); // * f32::consts::FRAC_PI_2;
    let quaternion = na::UnitQuaternion::new(axisangle);
    Madgwick::new_with_quat(sample_period, beta, quaternion)
}

fn imu_to_accelerometer(imu_data: (f32, f32, f32), external_forces: (f32, f32, f32)) -> Vector3<f32> {
    let (roll, pitch, _) = imu_data;
    let (external_x, external_y, external_z) = external_forces;
    let magnitude = G;
    let x = -magnitude * pitch.sin() + external_x;
    let y = magnitude * roll.sin() * pitch.cos() + external_y;
    let z = magnitude * roll.cos() * pitch.cos() + external_z;
    Vector3::new(x, y, z)
}

fn imu_to_magnetometer(imu_data: (f32, f32, f32), external_forces: (f32, f32, f32)) -> Vector3<f32> {
    let (_, pitch, yaw) = imu_data;
    let (external_x, external_y, external_z) = external_forces;
    let magnitude = EARTH_MAGNETIC_FIELD_MAGNITUDE;
    let x = magnitude * pitch.cos() * yaw.cos() + external_x;
    let y = magnitude * pitch.cos() * yaw.sin() + external_y;
    let z = magnitude * pitch.sin() + external_z;
    Vector3::new(x, y, z)
}

/// TODO: Document.
#[start]
fn _start(_: isize, _: *const *const u8) -> isize {
    let mut ahrs = create_ahrs();
    let mut stabilizer = create_stabilizer();

    // Simulated IMU sensor inputs
    let forward_acceleration = 1.0;
    let mut accelerometer = Vector3::new(forward_acceleration, 0.0, G);
    let mut gyroscope = Vector3::new(0.0, 0.0, 0.0);
    let mut magnetometer = Vector3::new(EARTH_MAGNETIC_FIELD_MAGNITUDE, 0.0, 0.0);

    // PID-stabilization parameters
    let initial_attitude = (45.0, 45.0, 45.0);
    let set_point = (10.0, 0.0, 10.0); // desired roll, pitch, yaw
    let dt = 1.0; // time step in seconds
    let low_throttle = false;

    let mut stdout = Stdout;
    let init_cycles = 1;
    for i in 0..init_cycles {
        let external_acceleration = (forward_acceleration, 0.0, 0.0);
        let accelerometer = imu_to_accelerometer(initial_attitude, external_acceleration);
        let gyroscope = Vector3::new(0.0, 0.0, 0.0);
        let gyroscope_radians = gyroscope * DEGREES_TO_RADIANS;
        let external_magnetism = (0.0, 0.0, 0.0);
        let magnetometer = imu_to_magnetometer(initial_attitude, external_magnetism);
        let imu_quaternion = ahrs
            .update(
                &gyroscope_radians,
                &accelerometer,
                &magnetometer,
            )
            .unwrap();
        let imu_attitude = imu_quaternion.euler_angles();
        let imu_attitude_degrees = (imu_attitude.0 * RADIANS_TO_DEGREES, imu_attitude.1 * RADIANS_TO_DEGREES, imu_attitude.2 * RADIANS_TO_DEGREES);
        if init_cycles == i+1 {
            writeln!(stdout, "init_cycles = {:.3}", init_cycles).ok();
            print_tuple3(&mut stdout, INDENT, HEADING_W, "Target:", initial_attitude);
            print_tuple3(&mut stdout, INDENT, HEADING_W, "IMU Attitude:", imu_attitude_degrees);
            print_vector3(&mut stdout, INDENT, HEADING_W, "Accelerometer:", accelerometer);
            print_vector3(&mut stdout, INDENT, HEADING_W, "Gyroscope:", gyroscope);
            print_vector3(&mut stdout, INDENT, HEADING_W, "Magnetometer:", magnetometer);
            writeln!(stdout).ok();
        }
    }

    writeln!(stdout, "                      Roll,    Pitch,      Yaw").ok();
    let mut t = 0.0;
    for _ in 0..=10 {
        let gyroscope_radians = gyroscope * DEGREES_TO_RADIANS;
        let imu_quaternion = ahrs
            .update(
                &gyroscope_radians,
                &accelerometer,
                &magnetometer,
            )
            .unwrap();
        let mut imu_attitude = imu_quaternion.euler_angles();
        let mut imu_attitude_degrees = (imu_attitude.0 * RADIANS_TO_DEGREES, imu_attitude.1 * RADIANS_TO_DEGREES, imu_attitude.2 * RADIANS_TO_DEGREES);

        // Perform the control computation
        let (roll_pid, pitch_pid, yaw_pid) =
            stabilizer.control(set_point, imu_attitude_degrees, (gyroscope.x, gyroscope.y, gyroscope.z), dt, low_throttle);

        // print results
        writeln!(stdout, "t = {:.3}, dt = {:.3}", t, dt).ok();
        print_tuple3(&mut stdout, INDENT, HEADING_W, "Set Point:", set_point);
        print_tuple3(&mut stdout, INDENT, HEADING_W, "IMU Attitude:", imu_attitude_degrees);
        print_vector3(&mut stdout, INDENT, HEADING_W, "Accelerometer:", accelerometer);
        print_vector3(&mut stdout, INDENT, HEADING_W, "Gyroscope:", gyroscope);
        print_vector3(&mut stdout, INDENT, HEADING_W, "Magnetometer:", magnetometer);
        print_tuple3(&mut stdout, INDENT, HEADING_W, "PID:", (roll_pid, pitch_pid, yaw_pid));

        // simulate response
        // X axis points from tail to nose
        // Y axis points from the left wing to right wing
        // Z axis points from top to bottom
        imu_attitude_degrees.0 += roll_pid * dt;
        imu_attitude_degrees.1 += pitch_pid * dt;
        imu_attitude_degrees.2 += yaw_pid * dt;
        imu_attitude = (imu_attitude_degrees.0 * DEGREES_TO_RADIANS, imu_attitude_degrees.1 * DEGREES_TO_RADIANS, imu_attitude_degrees.2 * DEGREES_TO_RADIANS);
        let external_acceleration = (forward_acceleration, 0.0, 0.0);
        accelerometer = imu_to_accelerometer(imu_attitude, external_acceleration);
        gyroscope.x = roll_pid;
        gyroscope.y = pitch_pid;
        gyroscope.z = yaw_pid;
        let external_magnetism = (0.0, 0.0, 0.0);
        magnetometer = imu_to_magnetometer(imu_attitude, external_magnetism);

        print_tuple3(&mut stdout, INDENT, HEADING_W, "IMU Attitude:", imu_attitude_degrees);

        t += dt;
    }

    0
}
