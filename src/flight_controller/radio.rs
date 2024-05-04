//! unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
//! unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

use parse_display::{FromStr, Display};

#[derive(Display, FromStr, PartialEq, Debug)]
#[display(" {ch_1} {ch_2} {ch_3} {ch_4} {ch_5} {ch_6}")]
pub struct RadioData {
    ch_1: Channel1Pwm,
    ch_2: Channel2Pwm,
    ch_3: Channel3Pwm,
    ch_4: Channel4Pwm,
    ch_5: Channel5Pwm,
    ch_6: Channel6Pwm,
}

#[derive(Display, FromStr, PartialEq, Debug)]
#[display("CH{channel}:{value}")]
pub struct Channel<const CHANNEL: u8, PWM> {
    value: PWM,
    channel: u8,
}

impl<const CHANNEL: u8, PWM> Channel<CHANNEL, PWM> {
    pub fn new(pwm_value: PWM) -> Self {
        Self {
            value: pwm_value,
            channel: CHANNEL.into(),
        }
    }
}

pub type ChannelPwm<const CHANNEL: u8> = Channel<CHANNEL, u32>;

pub type Channel1Pwm = Channel<1, u32>;
pub type Channel2Pwm = Channel<2, u32>;
pub type Channel3Pwm = Channel<3, u32>;
pub type Channel4Pwm = Channel<4, u32>;
pub type Channel5Pwm = Channel<5, u32>;
pub type Channel6Pwm = Channel<6, u32>;

pub type Channel1PwmPrev = Channel<1, u32>;
pub type Channel2PwmPrev = Channel<2, u32>;
pub type Channel3PwmPrev = Channel<3, u32>;
pub type Channel4PwmPrev = Channel<4, u32>;

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    // > Radio pwm values (expected: 1000 to 2000)
    pub const RADIO_DATA_SERIAL_MSG: &str = " CH1:1000 CH2:1200 CH3:1300 CH4:1400 CH5:1500 CH6:1600";

    #[test]
    fn test_radio_channels_data_parse() {
        let radio_data = RADIO_DATA_SERIAL_MSG.parse::<RadioData>().expect("Should parse radio channels data");

        dbg!(radio_data);
    }

}