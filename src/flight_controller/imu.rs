//! IMU
use std::primitive::str;

use parse_display::{Display, FromStr};

pub type AccX = Acceleration<X, f32>;
pub type AccY = Acceleration<Y, f32>;
pub type AccZ = Acceleration<Z, f32>;

pub type GyroX = Gyroscope<X, f32>;
pub type GyroY = Gyroscope<Y, f32>;
pub type GyroZ = Gyroscope<Z, f32>;

#[derive(Display, FromStr, Debug, PartialEq, Eq, Clone)]
#[display("Acc{axis}:{value}")]
pub struct Acceleration<AXIS: Axis, V> {
    value: V,
    axis: AXIS,
}

#[derive(Display, FromStr, Debug, PartialEq, Eq, Clone)]
#[display("Gyro{axis}:{value}")]
pub struct Gyroscope<AXIS: Axis, V> {
    #[from_str(default)]
    value: V,
    axis: AXIS,
}

pub trait Axis: core::fmt::Display {}

#[derive(Display, FromStr, Debug, PartialEq, Eq, Clone, Copy)]
#[display("X")]
pub struct X;
impl Axis for X {}

#[derive(Display, FromStr, Debug, PartialEq, Eq, Clone, Copy)]
#[display("Y")]
pub struct Y;
impl Axis for Y {}

#[derive(Display, FromStr, Debug, PartialEq, Eq, Clone, Copy)]
#[display("Z")]
pub struct Z;
impl Axis for Z {}

/// Acceleration
/// > Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
/// ```cpp
/// float AccX, AccY, AccZ;
/// float AccX_prev, AccY_prev, AccZ_prev;
/// ````
///
/// <https://www.arduino.cc/reference/en/language/variables/data-types/float>
#[derive(Display, FromStr, Debug, PartialEq, Clone)]
#[display("{x} {y} {z}")]
pub struct Acc {
    x: AccX,
    y: AccY,
    z: AccZ,
}

/// > Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
#[derive(Display, FromStr, Debug, PartialEq, Clone)]
#[display("{x} {y} {z}")]
pub struct Gyro {
    x: GyroX,
    y: GyroY,
    z: GyroZ,
}

// TODO: WTF is `F()`
#[cfg(test)]
pub mod tests {
    pub const GYRO_SERIAL_MSG: &str = "GyroX:4.22222 GyroY:230.0000 GyroZ:-100.123456789";
    pub const ACC_SERIAL_MSG: &str = "AccX:0.1111 AccY:1.999999 AccZ:1";

    use super::*;

    /// <https://github.com/nickrehm/dRehmFlight/blob/423806a7058e81e410d1ec1f7284a49555859112/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino#L1605-L1615>
    ///
    ///
    /// ```cpp
    /// Serial.print(F("GyroX:"));
    /// Serial.print(GyroX);
    /// Serial.print(F(" GyroY:"));
    /// Serial.print(GyroY);
    /// Serial.print(F(" GyroZ:"));
    /// Serial.println(GyroZ);
    /// ```
    #[test]
    fn test_gyroscope_parse() {
        let gyro = GYRO_SERIAL_MSG.parse::<Gyro>().expect("Should parse value");

        dbg!(gyro);
    }

    /// <https://github.com/nickrehm/dRehmFlight/blob/423806a7058e81e410d1ec1f7284a49555859112/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino#L1617-1627>
    ///
    /// ```cpp
    /// Serial.print(F("AccX:"));
    /// Serial.print(AccX);
    /// Serial.print(F(" AccY:"));
    /// Serial.print(AccY);
    /// Serial.print(F(" AccZ:"));
    /// Serial.println(AccZ);
    /// ```
    #[test]
    fn test_acceleration_parse() {
        let acceleration = ACC_SERIAL_MSG.parse::<Acc>().expect("Should parse value");

        dbg!(acceleration);
    }
}
