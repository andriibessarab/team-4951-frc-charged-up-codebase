package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;


/**
 *This class represents a gyroscope and provides methods to interact with it.
 */
public final class Gyroscope {
    private ADIS16470_IMU gyro;

    
    /**
     * Constructs a new gyroscope object.
     */
    public Gyroscope() {
        this.gyro = new ADIS16470_IMU();
    }


    /**
     * Calibrates the gyroscope.
     */
    public final void calibrate() {
        gyro.calibrate();
    }


    /**
     * Resets the gyroscope.
     */
    public final void reset() {
        gyro.reset();
    }


    /**
     * Returns the angle of the yaw axis in degrees.
     * @return the angle of the yaw axis in degrees
     */
    public final double getAngle() {
        return gyro.getAngle();
    }


    /**
     * Calculates and returns the current pitch angle of the robot using the arctangent function and data from the gyro's accelerometer.
     * @return The current pitch angle in degrees.
     */
    public final double getPitch() {
        // Calculate the pitch angle 
        return Math.atan2(gyro.getAccelY(), gyro.getAccelZ()) * 180 / Math.PI;
    }


    /**
     * Calculates and returns the current thresholded pitch angle of the robot using the arctangent function and data from the gyro's accelerometer.
     * @return The current thresholded pitch angle in degrees.
     */
    public final double getThresholdedPitch() {
        // Calculate the pitch angle 
        return Math.atan2(gyro.getAccelY(), gyro.getAccelZ()) * 180 / Math.PI > 5 ? Math.atan2(gyro.getAccelY(), gyro.getAccelZ()) * 180 / Math.PI : 0;
    }


    /**
     * Returns the current rate of rotation of the gyro.
     * @return The current rate of rotation in degrees per second.
     */
    public final double getRate() {
        return -gyro.getRate();
    }


    /**
     * Returns the angle of the yaw axis as a Rotation2d object.
     * @return the angle of the yaw axis as a Rotation2d object
     */
    public final Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }
}
