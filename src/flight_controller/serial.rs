use parse_display::{Display, FromStr};

use super::{
    imu::{Acc, Gyro},
    radio::RadioData,
};

/// Default:
/// `Serial.begin(500000); // USB serial`
///
/// <https://github.com/nickrehm/dRehmFlight/blob/423806a7058e81e410d1ec1f7284a49555859112/Versions/dRehmFlight_Teensy_BETA_1.3/dRehmFlight_Teensy_BETA_1.3.ino#L309>
///
pub const SERIAL_BAUD_RATE: u32 = 500_000;

#[derive(Display, FromStr, PartialEq, Debug)]
pub enum SerialData {
    #[display("{0}")]
    ImuAcceleration(Acc),
    #[display("{0}")]
    ImuGyroscope(Gyro),
    #[display("{0}")]
    RadioData(RadioData),
}

#[cfg(test)]
pub mod test {
    use core::fmt::Write;

    use crate::flight_controller::{
        imu::tests::{ACC_SERIAL_MSG, GYRO_SERIAL_MSG},
        radio::{tests::RADIO_DATA_SERIAL_MSG, RadioData},
    };

    use super::*;

    #[test]
    fn test_all_known_serial_sentences() {
        let all = vec![GYRO_SERIAL_MSG, ACC_SERIAL_MSG, RADIO_DATA_SERIAL_MSG];

        let sentences = all
            .iter()
            .try_fold(String::new(), |mut string, item| {
                writeln!(&mut string, "{item}")?;

                Ok::<String, std::fmt::Error>(string)
            })
            .expect("Should concatenate on different lines");

        let sentences_parsed = sentences
            .lines()
            .map(|line| line.parse::<SerialData>())
            .collect::<Result<Vec<SerialData>, parse_display::ParseError>>()
            .expect("Should not fail parsing sentences");

        dbg!(sentences_parsed);
    }
}
