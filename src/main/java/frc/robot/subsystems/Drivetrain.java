/**
 * This class represents the subsystem responsible for controlling the robot's drivetrain.
 * It uses a mecanum drive to allow for multi-axis and omnidirectional movement.
 * 
 * It includes methods for driving the robot using both robot-oriented and field-oriented control.
 * 
 * The class relies on the following components:
 * - Motor class: for controlling each motor on the robot
 * - Gyroscope class: for measuring the robot's orientation
 * - MecanumDriveKinematics class: for calculating kinematics based on wheel location
 * - MecanumDriveOdometry class: for calculating the robot's position on the field
 * 
 * @see Motor
 * @see Gyroscope
 * @see MecanumDriveKinematics
 * @see MecanumDriveOdometry
 */

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;

import frc.robot.RobotMap;


public final class Drivetrain {
    // Create mecanum drive kinematics object based on motor location
    private final static MecanumDriveKinematics kenamatics = new MecanumDriveKinematics(RobotMap.frontLeftMotor.getLocation(), RobotMap.frontRightMotor.getLocation(), RobotMap.rearLeftMotor.getLocation(), RobotMap.rearRightMotor.getLocation());
    
    // Create mecanum drive odometry object based on kinematics and initial conditions
    private final static MecanumDriveOdometry odometry = new MecanumDriveOdometry(
        kenamatics,
        RobotMap.gyro.getRotation2D(),
        new MecanumDriveWheelPositions(
            RobotMap.frontLeftMotor.getEncoderPosition(), RobotMap.frontRightMotor.getEncoderPosition(),
            RobotMap.rearLeftMotor.getEncoderPosition(), RobotMap.rearRightMotor.getEncoderPosition()
        )
    );


    /**
     * This private method sets the speed for each motor on the robot.
     * 
     * @param fl speed for the front left motor
     * @param fr speed for the front right motor
     * @param bl speed for the back left motor
     * @param br speed for the back right motor
     */
    private final static void setMotorSpeeds(double fl, double fr, double bl, double br) {
        RobotMap.frontLeftMotor.setSpeed(fl);
        RobotMap.frontLeftMotor.setSpeed(fr);
        RobotMap.frontLeftMotor.setSpeed(bl);
        RobotMap.frontLeftMotor.setSpeed(br);
    }


    /**
     * This method drives the robot using single-axis control.
     * 
     * @param x x-axis movement speed
     * @param y y-axis movement speed
     * @param z z-axis movement speed
     */
    public final static void driveSingleAxis(double x, double y, double z) {
        if (Math.abs(y) > Math.abs(x) && Math.abs(y) > Math.abs(z)) { // Y-Axis Motion
            setMotorSpeeds(y, y, y, y);
        } else if (Math.abs(x) > Math.abs(y) && Math.abs(x) > Math.abs(z)) { // X-Axis Motion
            if(x > 0) {
                setMotorSpeeds(x*1.1, x*-0.95, x*-1.1, x);
                if(Math.abs(x) >= 0.5) {
                    setMotorSpeeds(x*1.1, x*-0.95, x*-1.1, x);
                } else if(Math.abs(x) > 0.35) {
                    setMotorSpeeds(x*0.9, x*-0.9, x*-1.1, x);

                }
            } else if(x < 0) {
                if(Math.abs(x) >= 0.5) {
                    setMotorSpeeds(x*1.05, x*-1, x*-1, x);
                } else if(Math.abs(x) > 0.35) {
                    setMotorSpeeds(x*1.1, x*-1, x*-1.1, x);
                }
            }
        } else if (Math.abs(z) > Math.abs(y) && Math.abs(z) > Math.abs(x)) { // Z-Axis Movement
            setMotorSpeeds(z, -z, z, -z);
        } else {
            setMotorSpeeds(0, 0, 0, 0);
        }
    }


    /**
     * This method provides field-oriented mecanum drive. It calculates the power to each motor based on the
     * robot's current heading as determined by a gyro, and the input speed and rotation values. It also applies a
     * correction factor to the ySpeed to counteract imperfect strafing. The resulting motor power values are sent to
     * the setMotorSpeeds() method to be applied to the physical motors.
     * @param xSpeed The desired speed in the x direction.
     * @param ySpeed The desired speed in the y direction.
     * @param zRot The desired rotation speed around the z axis.
     * @param gyroAngle The current heading of the robot as measured by a gyro, in degrees.
    */
    public final static void driveFieldOriented(double xSpeed,double ySpeed, double zRot, double gyroAngle) {
        ySpeed = ySpeed * 1.1; // Counteract imperfect strafing

        // Calculate denominator
        double botDir = (Math.PI / 180) * gyroAngle;

        // Rotate the movement direction counter to the bot's rotation
        double xRot = xSpeed * Math.cos(-botDir) - ySpeed * Math.sin(-botDir);
        double yRot = xSpeed * Math.sin(-botDir) + ySpeed * Math.cos(-botDir);

        // Calculate deniminator
        double denominator = Math.max(Math.abs(yRot) + Math.abs(xRot) + Math.abs(zRot), 1);

        // Calculate motors power
        double rearLeftPower = (yRot - xRot + zRot) / denominator;
        double frontLeftPower = (yRot + xRot + zRot) / denominator;
        double frontRightPower = (yRot - xRot - zRot) / denominator;
        double rearRightPower = (yRot + xRot - zRot) / denominator;

        // Send power to motors
        setMotorSpeeds(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }


    /**
     * This method provides robot-oriented mecanum drive. It calculates the power to each motor based solely on the input
     * speed and rotation values. It also applies a correction factor to the ySpeed to counteract imperfect strafing.
     * The resulting motor power values are sent to the setMotorSpeeds() method to be applied to the physical motors.
     * @param xSpeed The desired speed in the x direction.
     * @param ySpeed The desired speed in the y direction.
     * @param zRot The desired rotation speed around the z axis.
     */
    public final static void driveRobotOriented(double xSpeed,double ySpeed, double zRot) {
        ySpeed = ySpeed * 1.1; // Counteract imperfect strafing
    
        // Calculate deniminator
        double denominator = Math.max(Math.abs(ySpeed) + Math.abs(xSpeed) + Math.abs(zRot), 1);
    
        // Calculate motors power
        double rearLeftPower = (ySpeed - xSpeed + zRot) / denominator;
        double frontLeftPower = (ySpeed + xSpeed + zRot) / denominator;
        double frontRightPower = (ySpeed - xSpeed - zRot) / denominator;
        double rearRightPower = (ySpeed + xSpeed - zRot) / denominator;
    
        // Send power to motors
        setMotorSpeeds(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }
}
