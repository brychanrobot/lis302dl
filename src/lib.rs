/*
Copyright 2020 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#![no_std]
#![allow(non_camel_case_types)]

use accelerometer::RawAccelerometer;
use core::fmt::Debug;
use embedded_hal;
use embedded_hal::digital::v2::OutputPin;

const EXPECTED_DEVICE_ID: u8 = 0x3B;

const WHO_AM_I_REGISTER: u8 = 0x0F;
const X_OUTPUT_REGISTER: u8 = 0x29;
const Y_OUTPUT_REGISTER: u8 = 0x2B;
const Z_OUTPUT_REGISTER: u8 = 0x2D;

const CONTROL_REGISTER_1: u8 = 0x20;
const DATA_RATE_100_HZ: u8 = 0x00;
const DATA_RATE_400_HZ: u8 = 0x80;
const POWER_DOWN_MODE: u8 = 0x00;
const ACTIVE_MODE: u8 = 0x40;
const SCALE_PLUS_MINUS_2G: u8 = 0x00;
const SCALE_PLUS_MINUS_8G: u8 = 0x20;
const Z_ENABLE: u8 = 0x04;
const Y_ENABLE: u8 = 0x02;
const X_ENABLE: u8 = 0x01;

const READ_FLAG: u8 = 0x80;

const SCALE: f32 = 4.6 / 256.0; // When multiplied by the output give the acceleration in g's

pub enum PowerMode {
    Active,
    PowerDown,
}

pub enum Scale {
    PlusMinus2G,
    PlusMinus8G,
}

pub enum DataRate {
    Rate100Hz,
    Rate400Hz,
}

pub struct Config {
    pub power_mode: PowerMode,
    pub scale: Scale,
    pub data_rate: DataRate,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            power_mode: PowerMode::Active,
            scale: Scale::PlusMinus2G,
            data_rate: DataRate::Rate400Hz,
        }
    }
}

pub struct Lis302Dl<Spi, CsPin> {
    spi: Spi,
    chip_select: CsPin,
    config: Config,
}

impl<Spi, SpiError, CsPin, PinError> Lis302Dl<Spi, CsPin>
where
    Spi: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
{
    pub fn new(spi: Spi, chip_select: CsPin, config: Config) -> Self {
        let mut lis302dl = Lis302Dl {
            spi,
            chip_select,
            config,
        };

        if lis302dl.get_device_id() != EXPECTED_DEVICE_ID {
            // TODO: error
        }

        lis302dl.set_control_register_1();

        lis302dl
    }

    fn read_byte(&mut self, address: u8) -> u8 {
        let mut bytes = [READ_FLAG | address, 0x0];
        self.chip_select.set_low().ok();
        self.spi.transfer(&mut bytes).ok();
        self.chip_select.set_high().ok();
        bytes[1]
    }

    fn write_byte(&mut self, address: u8, value: u8) {
        let mut request = [address, value];
        self.chip_select.set_low().ok();
        self.spi.write(&mut request).ok();
        self.chip_select.set_high().ok();
    }

    fn get_device_id(&mut self) -> u8 {
        self.read_byte(WHO_AM_I_REGISTER)
    }

    fn set_control_register_1(&mut self) {
        let mut control_byte = X_ENABLE | Y_ENABLE | Z_ENABLE;
        control_byte |= match self.config.power_mode {
            PowerMode::Active => ACTIVE_MODE,
            PowerMode::PowerDown => POWER_DOWN_MODE,
        };
        control_byte |= match self.config.scale {
            Scale::PlusMinus2G => SCALE_PLUS_MINUS_2G,
            Scale::PlusMinus8G => SCALE_PLUS_MINUS_8G,
        };
        control_byte |= match self.config.data_rate {
            DataRate::Rate100Hz => DATA_RATE_100_HZ,
            DataRate::Rate400Hz => DATA_RATE_400_HZ,
        };
        self.write_byte(CONTROL_REGISTER_1, control_byte);
    }
}

impl<Spi, SpiError, CsPin, PinError> accelerometer::RawAccelerometer<accelerometer::vector::I8x3>
    for Lis302Dl<Spi, CsPin>
where
    Spi: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
{
    type Error = SpiError;
    fn accel_raw(
        &mut self,
    ) -> Result<accelerometer::vector::I8x3, accelerometer::Error<Self::Error>> {
        let x = self.read_byte(X_OUTPUT_REGISTER);
        let y = self.read_byte(Y_OUTPUT_REGISTER);
        let z = self.read_byte(Z_OUTPUT_REGISTER);
        Ok(accelerometer::vector::I8x3::new(
            i8::from_le_bytes([x]),
            i8::from_le_bytes([y]),
            i8::from_le_bytes([z]),
        ))
    }
}

impl<Spi, SpiError, CsPin, PinError> accelerometer::Accelerometer for Lis302Dl<Spi, CsPin>
where
    Spi: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CsPin: OutputPin<Error = PinError>,
    SpiError: Debug,
{
    type Error = SpiError;
    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Self::Error>> {
        match self.config.data_rate {
            DataRate::Rate100Hz => Ok(100.0),
            DataRate::Rate400Hz => Ok(400.0),
        }
    }

    fn accel_norm(
        &mut self,
    ) -> Result<accelerometer::vector::F32x3, accelerometer::Error<Self::Error>> {
        let raw_acceleration: accelerometer::vector::I8x3 = self.accel_raw().unwrap();
        Ok(accelerometer::vector::F32x3::new(
            raw_acceleration.x as f32 * SCALE,
            raw_acceleration.y as f32 * SCALE,
            raw_acceleration.z as f32 * SCALE,
        ))
    }
}
