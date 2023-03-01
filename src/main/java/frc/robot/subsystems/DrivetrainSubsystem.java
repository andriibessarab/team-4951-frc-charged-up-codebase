package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotMap;
import frc.robot.utils.Gyroscope;
import frc.robot.utils.Motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * This class defines constants that are used in drivetrain.
     */
    public static final class Constants {
        // Constants for Trajecotories (need to obtain them through sysid and measurements)
        private static double ksVolts;
        private static double kvVoltSecondsPerMinuite;
        private static double kaVoltSecondsSquaredPerMinuite;
        private static double kpDriveVel;
        private static double kTrackWidthMeters; // horizontal distance between two wheels
        private static double kGearRatioInches;
        private static double kWheelRadiusInches; // radius of a wheel in inches
        private static final double kLinearDistanceConversionFactor = Units.inchesToMeters(1 / (kGearRatioInches*2*Math.PI*Units.inchesToMeters(kWheelRadiusInches)) * 10);

        // Constants & variables for balancing
        private static final double kProportionalGain = 0.03; // Proportional gain
        private static final double kIntegralGain = 0.00; // Integral gain
        private static final double kDerivativeGain = 0.00; // Derivative gain
        private static final double kToleranceDegrees = 2.0f; // Tolerance for gyro angle
        private static final double kMaxOutput = 0.5; // Maximum output
        private static final double kMinOutput = -0.5; // Minimum output
    }

    
    // The gyroscope object used for detecting and measuring the robot's rotation.
	public Gyroscope gyro;

	// The motor objects used for controlling the robot's drivetrain
	public Motor rearLeftMotor;
	public Motor rearRightMotor;
	public Motor frontLeftMotor;
	public Motor frontRightMotor;
    
	private MecanumDriveKinematics kinematics;  // Create mecanum drive kinematics object based on motor location
	private MecanumDriveOdometry odometry;  // Mecanum drive odometry object based on kinematics and initial conditions

    // PID Controller Variables
    private double integralError = 0.0f; // Integral error
    private double previousError = 0.0f; // Previous error

    /**
     * Constructor for the DrivetrainSubsystem class.
     *
     */
    public DrivetrainSubsystem() {
        gyro = new Gyroscope();

        gyro.reset();
        gyro.calibrate();

        rearLeftMotor = new Motor(RobotMap.REAR_LEFT_MOTOR_PWM_PIN, 0, 0);
        rearRightMotor = new Motor(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN, 0, 0);
        frontLeftMotor = new Motor(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN, 0, 0);
        frontRightMotor = new Motor(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN, 0, 0);

        frontLeftMotor.restoreMotorToFactoryDefaults();
        frontRightMotor.restoreMotorToFactoryDefaults();
        rearLeftMotor.restoreMotorToFactoryDefaults();
        rearRightMotor.restoreMotorToFactoryDefaults();

        resetEncoders();

        frontLeftMotor.setEncoderPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
        frontRightMotor.setEncoderPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
        rearLeftMotor.setEncoderPositionConversionFactor(Constants.kLinearDistanceConversionFactor);
        rearRightMotor.setEncoderPositionConversionFactor(Constants.kLinearDistanceConversionFactor);

        frontLeftMotor.setEncoderVelocityConversionFactor(Constants.kLinearDistanceConversionFactor / 60);
        frontRightMotor.setEncoderVelocityConversionFactor(Constants.kLinearDistanceConversionFactor / 60);
        rearLeftMotor.setEncoderVelocityConversionFactor(Constants.kLinearDistanceConversionFactor / 60);
        rearRightMotor.setEncoderVelocityConversionFactor(Constants.kLinearDistanceConversionFactor / 60);

        kinematics = new MecanumDriveKinematics(frontLeftMotor.getLocation(), frontRightMotor.getLocation(), rearLeftMotor.getLocation(), rearRightMotor.getLocation());
        odometry = new MecanumDriveOdometry(kinematics, gyro.getRotation2D(), getMecanumDriveWheelPositions());
    
        resetOdometry(new Pose2d());
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    /**
     * Resets the encoder counts of a given motor to zero.
     * @param motor the motor to reset the encoder counts of
     */
    public final void resetEncoders() {
        frontLeftMotor.resetEncoder();
        frontRightMotor.resetEncoder();
        rearLeftMotor.resetEncoder();
        rearRightMotor.resetEncoder();
    }


    /**
     * Resets the robot's odometry to a specified starting pose.
     * @param startingPose the pose to set the robot's odometry to
     */
    public final void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(gyro.getRotation2D(), getMecanumDriveWheelPositions(), pose);
    }


    /**
     * Updates the robot's odometry information using the current sensor readings.
     * This method calculates the robot's position and orientation based on readings from the
     * gyro and the mecanum drive motor encoders.
     * @see <a href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/odometry.html">WPILib Odometry documentation</a>
    */
    public final void updateOdometry() {
        odometry.update(
            gyro.getRotation2D(),
		    getMecanumDriveWheelPositions()
        );
    }


    /**
     * Gets the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public final Pose2d getPose() {
        return odometry.getPoseMeters();
    }


    /**
     * Gets the positions of the mecanum drive wheels.
     *
     * @return The positions of the mecanum drive wheels.
     */
    public final MecanumDriveWheelPositions getMecanumDriveWheelPositions() {
        return new MecanumDriveWheelPositions(frontLeftMotor.getEncoderPosition(), frontRightMotor.getEncoderPosition(), rearLeftMotor.getEncoderPosition(), rearRightMotor.getEncoderPosition());
    }


    /**
     * This private method sets the speed for each motor on the robot.
     * 
     * @param fl speed for the front left motor
     * @param fr speed for the front right motor
     * @param bl speed for the back left motor
     * @param br speed for the back right motor
     */
    private final void setMotorSpeeds(double fl, double fr, double bl, double br) {
        frontLeftMotor.setSpeed(fl);
        frontRightMotor.setSpeed(fr);
        rearLeftMotor.setSpeed(bl);
        rearRightMotor.setSpeed(br);
    }


    /**
     * This method drives the robot using single-axis control.
     * 
     * @param x x-axis movement speed
     * @param y y-axis movement speed
     * @param z z-axis movement speed
     */
    public final void driveSingleAxis(double x, double y, double z) {
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
    public final void driveFieldOriented(double xSpeed,double ySpeed, double zRot, double gyroAngle) {
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
    public final void driveRobotOriented(double xSpeed,double ySpeed, double zRot) {
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
     * Balances the robot on a balancing station using a PID controller.
     */
    public final boolean balanceOnStation() {
          double angle = gyro.getPitch();
          double error = -angle; // Negative because we want to balance on the opposite side of the gyro angle
          if (Math.abs(error) < Constants.kToleranceDegrees) { // If within tolerance, stop
            setMotorSpeeds(0, 0, 0, 0);
            return true;
          }
          double output = Constants.kProportionalGain * error + Constants.kIntegralGain * integralError + Constants.kDerivativeGain * (error - previousError); // PID calculation
          output = Math.max(Constants.kMinOutput, Math.min(Constants.kMaxOutput, output)); // Clamp output to within limits
          setMotorSpeeds(output, output, output, output); // Set the motor speeds based on the PID output
          // Possible add delay to wait a short amount of time before checking again
          integralError += error; // Update integral error
          previousError = error; // Update previous error
          return false;
    }
}
