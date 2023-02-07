package frc.robot.utils;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public final class Gyroscope {

    private final static ADIS16470_IMU gyro = new ADIS16470_IMU(); // ADIS16470 Gyroscope

    // Calibrate gyro
    public final static void calibrate() {
        gyro.calibrate();
    }

    // Reset gyro
    public final static void reset() {
        gyro.reset();
    }

    // Get yaw axis angle in degrees
    public final static double getAngle() {
        return gyro.getAngle();
    }
}
