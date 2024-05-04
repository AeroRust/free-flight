
/// Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
///
/// - [ ] printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
/// - [ ] printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
/// - [x] printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
/// - [x] printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
/// - [ ] printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
/// - [ ] printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
/// - [ ] printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
/// - [ ] printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
/// - [ ] printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
/// - [ ] printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
pub mod flight_controller {
    pub mod serial;
    pub mod imu;
    pub mod radio;
}


#[cfg(test)]
mod tests {
    use super::*;
}
