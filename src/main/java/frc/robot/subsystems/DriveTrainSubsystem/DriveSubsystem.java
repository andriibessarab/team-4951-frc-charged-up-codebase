package frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Gyroscope;

public abstract class DriveSubsystem extends SubsystemBase {

    
    // The gyroscope object used for detecting and measuring the robot's rotation.
    public Gyroscope gyro;

    // PID Controller Variables
    private double integralError = 0.0f; // Integral error
    private double previousError = 0.0f; // Previous error


        /**
     * This class defines constants that are used in drivetrain.
     */
    public static class DrivetrainConstants {
        // Constants & variables for balancing
        private static final double kProportionalGain = 0.03; // Proportional gain
        private static final double kIntegralGain = 0.00; // Integral gain
        private static final double kDerivativeGain = 0.00; // Derivative gain
        private static final double kToleranceDegrees = 1.0f; // Tolerance for gyro angle
        private static final double kMaxOutput = 0.5; // Maximum output
        private static final double kMinOutput = -0.5; // Minimum output
    }
    
    /**
     * This method drives the robot using single-axis control.
     * 
     * @param xSpeed x-axis movement speed
     * @param y      y-axis movement speed
     * @param z      z-axis movement speed
     */
    public void driveSingleAxis(double xSpeed, double y, double z) {
        if (Math.abs(y) > Math.abs(xSpeed) && Math.abs(y) > Math.abs(z)) { // Y-Axis Motion
            setMotorSpeeds(y, y, y, y);
        } else if (Math.abs(xSpeed) > Math.abs(y) && Math.abs(xSpeed) > Math.abs(z)) { // X-Axis Motion
            if (xSpeed > 0) {
                setMotorSpeeds(xSpeed * 1.1, xSpeed * -0.95, xSpeed * -1.1, xSpeed);
                if (Math.abs(xSpeed) >= 0.5) {
                    setMotorSpeeds(xSpeed * 1.1, xSpeed * -0.95, xSpeed * -1.1, xSpeed);
                } else if (Math.abs(xSpeed) > 0.35) {
                    setMotorSpeeds(xSpeed * 0.9, xSpeed * -0.9, xSpeed * -1.1, xSpeed);

                }
            } else if (xSpeed < 0) {
                if (Math.abs(xSpeed) >= 0.5) {
                    setMotorSpeeds(xSpeed * 1.05, xSpeed * -1, xSpeed * -1, xSpeed);
                } else if (Math.abs(xSpeed) > 0.35) {
                    setMotorSpeeds(xSpeed * 1.1, xSpeed * -1, xSpeed * -1.1, xSpeed);
                }
            }
        } else if (Math.abs(z) > Math.abs(y) && Math.abs(z) > Math.abs(xSpeed)) { // Z-Axis Movement
            setMotorSpeeds(z, -z, z, -z);
        } else {
            setMotorSpeeds(0, 0, 0, 0);
        }
    }

    /**
     * This method provides field-oriented mecanum drive. It calculates the power to
     * each motor based on the
     * robot's current heading as determined by a gyro, and the input speed and
     * rotation values. It also applies a
     * correction factor to the ySpeed to counteract imperfect strafing. The
     * resulting motor power values are sent to
     * the setMotorSpeeds() method to be applied to the physical motors.
     * 
     * @param xSpeed    The desired speed in the x direction.
     * @param ySpeed    The desired speed in the y direction.
     * @param zRot      The desired rotation speed around the z axis.
     * @param gyroAngle The current heading of the robot as measured by a gyro, in
     *                  degrees.
     */
    public void driveFieldOriented(double xSpeed, double ySpeed, double zRot, double gyroAngle) {
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
     * This method provides robot-oriented mecanum drive. It calculates the power to
     * each motor based solely on the input
     * speed and rotation values. It also applies a correction factor to the ySpeed
     * to counteract imperfect strafing.
     * The resulting motor power values are sent to the setMotorSpeeds() method to
     * be applied to the physical motors.
     * 
     * @param xSpeed The desired speed in the x direction.
     * @param ySpeed The desired speed in the y direction.
     * @param zRot   The desired rotation speed around the z axis.
     */
    public void driveRobotOriented(double xSpeed, double ySpeed, double zRot) {
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

    /**
     * Drives a robot with mecanum wheels based on joystick input.
     * Calculates the appropriate motor powers for each wheel to achieve
     * smooth and accurate movement.
     *
     * @param xSpeed the x component of the joystick input (-1.0 to 1.0)
     * @param ySpeed the y component of the joystick input (-1.0 to 1.0)
     * @param zRot   the rotation component of the joystick input (-1.0 to 1.0)
     * 
     * @see <a href="https://www.youtube.com/watch?v=gnSW2QpkGXQ">This video</a> for
     *      a demonstration of mecanum wheel drive.
     */
    public void driveMecanum(double xSpeed, double ySpeed, double zRot) {
        // Calculate the angle and magnitude of the joystick input
        double theta = Math.atan2(ySpeed, xSpeed);
        double power = Math.hypot(xSpeed, ySpeed);

        // Calculate the sine and cosine of the angle, offset by 45 degrees
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Calculate the motor powers for each wheel based on the joystick input
        double leftFront = power * cos / max + zRot;
        double rightFront = power * sin / max - zRot;
        double leftRear = power * sin / max + zRot;
        double rightRear = power * cos / max - zRot;

        // Scale the motor powers if necessary to avoid exceeding the maximum power
        if ((power + Math.abs(zRot)) > 1) {
            leftFront /= power + Math.abs(zRot);
            rightFront /= power + Math.abs(zRot);
            leftRear /= power + Math.abs(zRot);
            rightRear /= power + Math.abs(zRot);
        }

        // Set the motor speeds to achieve the desired movement
        setMotorSpeeds(leftFront, rightFront, leftRear, rightRear);
    }

    /**
     * Balances the robot on a balancing station using a PID controller.
     */
    //TODO: try to use sparkmaxPIDController
    public boolean balanceOnStation() {
        double angle = gyro.getPitch();//TODO: not thresholded
        double error = -angle; // Negative because we want to balance on the opposite side of the gyro angle
        if (Math.abs(error) < DrivetrainConstants.kToleranceDegrees) { // If within tolerance, stop
            setMotorSpeeds(0, 0, 0, 0);
            return true;
        }
        double output = DrivetrainConstants.kProportionalGain * error
                + DrivetrainConstants.kIntegralGain * integralError
                + DrivetrainConstants.kDerivativeGain * (error - previousError); // PID calculation
        output = Math.max(DrivetrainConstants.kMinOutput, Math.min(DrivetrainConstants.kMaxOutput, output)); // Clamp
                                                                                                             // output
                                                                                                             // to
                                                                                                             // within
                                                                                                             // limits
        setMotorSpeeds(output, output, output, output); // Set the motor speeds based on the PID output
        // Possible add delay to wait a short amount of time before checking again
        integralError += error; // Update integral error
        previousError = error; // Update previous error
        return false;
    }

    abstract void setMotorSpeeds(double fl, double fr, double bl, double br);
}
