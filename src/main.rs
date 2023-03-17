//! Prima prova

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use fugit::{HertzU32, RateExtU32};

use cortex_m::delay::Delay;

use rp_pico as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c::I2C,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use mpu6050::*;

use ahrs::{Ahrs, Madgwick};

#[bsp::entry]
fn main() -> ! {
    info!("Program start");

    // Get RP2040 peripherals
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq: HertzU32 = 12.MHz();
    let clocks = init_clocks_and_plls(
        external_xtal_freq.to_Hz(),
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Setup I2C
    let i2c = I2C::i2c1(
        pac.I2C1,
        pins.gpio18.into_mode(), // SDA
        pins.gpio19.into_mode(), // SCL
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    // Setup MPU
    let address_mpu = 0x68;
    let mut mpu = Mpu6050::new_with_addr_and_sens(
        i2c,
        address_mpu,
        device::AccelRange::G8,
        device::GyroRange::D1000,
    );

    match mpu.init(&mut delay) {
        Ok(_) => {
            info!("Setup MPU-6050 seccessful!");
        }
        Err(err) => match err {
            Mpu6050Error::InvalidChipId(_) => {
                error!("MPU-6050 not found at address {=u8:#X}", address_mpu);
                defmt::panic!();
            }
            Mpu6050Error::I2c(_) => {
                error!("I2C error while setting up MPU-6050");
                defmt::panic!();
            }
        },
    }

    // Use Madgwick algorithm to estimate attitude (roll-pitch-yaw)
    let mut ahrs = Madgwick::new(0.010f32, 0.1f32);

    loop {
        // Get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro().unwrap();

        // Get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc().unwrap();

        // Update the IMU calculation
        let quat = ahrs.update_imu(&gyro, &acc).unwrap();

        // Convert quaternion to euler angles
        let (roll, pitch, yaw) = quat.euler_angles();

        // Print roll-pitch-yaw converted in degrees (from radians)
        println!(
            "RPY: {:?} {:?} {:?}",
            roll / PI_180,
            pitch / PI_180,
            yaw / PI_180
        );
    }
}
