package frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.utils.Gyroscope;
import frc.robot.utils.Motor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class represents the subsystem responsible for controlling the robot's
 * drivetrain.
 * It uses a mecanum drive to allow for multi-axis and omnidirectional movement.
 * 
 * It includes methods for driving the robot using both robot-oriented and
 * field-oriented control.
 * 
 * The class relies on the following components:
 * - Motor class: for controlling each motor on the robot
 * - Gyroscope class: for measuring the robot's orientationdifferential
 * 
 * @see Motor
 * @see Gyroscope
 * @see MecanumDriveKinematics
 * @see MecanumDriveOdometry
 */
public class SparkMax extends SubsystemBase {
    /**
     * This class defines constants that are used in drivetrain.
     */
    public static final class DrivetrainConstants {
        // Constants for Trajecotories (need to obtain them through sysid and
        // measurements)
        public static double ksVolts;
        public static double kvVoltSecondsPerMinuite;
        public static double kaVoltSecondsSquaredPerMinuite;
        public static double kpDriveVel;

        public static double kTrackWidthMeters = Units.inchesToMeters(20.5); // horizontal distance between two wheels
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);

        // Values obtained thrrough WILIB documentation
        public static final double kMaxSpeedMeterserSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 23;
        public static final double kRamseteZeta = 0.7;

        private static double kGearRatio = 10.71;
        private static double kWheelRadiusInches = 2.75; // radius of wheels
        private static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI *
                        Units.inchesToMeters(kWheelRadiusInches)) * 10));

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

    private MotorControllerGroup leftControllerGroup;
    private MotorControllerGroup rightControllerGroup;

    private DifferentialDrive m_differentialDrive;
    private DifferentialDriveOdometry m_odometry;

    // PID Controller Variables
    private double integralError = 0.0f; // Integral error
    private double previousError = 0.0f; // Previous error

    /**
     * Constructor for the DrivetrainSubsystem class.
     *
     */
    public SparkMax() {
        gyro = new Gyroscope();

        gyro.reset();
        gyro.calibrate();

        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2D(), frontLeftMotor.getEncoderPosition(),
                frontRightMotor.getEncoderPosition());

        rearLeftMotor = new Motor(RobotMap.REAR_LEFT_MOTOR_PWM_PIN);
        rearRightMotor = new Motor(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN);
        frontLeftMotor = new Motor(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN);
        frontRightMotor = new Motor(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN);

        leftControllerGroup = new MotorControllerGroup(frontLeftMotor.getMotorInstance(),
                rearLeftMotor.getMotorInstance());
        rightControllerGroup = new MotorControllerGroup(frontRightMotor.getMotorInstance(),
                rearRightMotor.getMotorInstance());

        m_differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);

        frontLeftMotor.restoreMotorToFactoryDefaults();
        frontRightMotor.restoreMotorToFactoryDefaults();
        rearLeftMotor.restoreMotorToFactoryDefaults();
        rearRightMotor.restoreMotorToFactoryDefaults();

        leftControllerGroup.setInverted(false);
        rightControllerGroup.setInverted(false);

        resetEncoders();

        frontLeftMotor.setEncoderPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        frontRightMotor.setEncoderPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        frontLeftMotor.setEncoderVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);
        frontRightMotor.setEncoderVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);

        resetOdometry(new Pose2d());
    }

    @Override
    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("Left-encoder value(meters)", frontLeftMotor.getEncoderPosition());
        SmartDashboard.putNumber("Right-encoder value(meters)", frontLeftMotor.getEncoderPosition());
        SmartDashboard.putNumber("Gyro heading", gyro.getAngle());
    }

    /**
     * Resets the encoder counts of a given motor to zero
     */
    public final void resetEncoders() {
        frontLeftMotor.resetEncoder();
        frontRightMotor.resetEncoder();
    }

    /**
     * Resets the robot's odometry to a specified starting pose.
     * 
     * @param startingPose the pose to set the robot's odometry to
     */
    public final void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(gyro.getRotation2D(), frontLeftMotor.getEncoderPosition(),
                frontRightMotor.getEncoderPosition(), pose);
    }

    /**
     * Updates the robot's odometry information using the current sensor readings.
     * This method calculates the robot's position and orientation based on readings
     * from the
     * gyro and the mecanum drive motor encoders.
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/odometry.html">WPILib
     *      Odometry documentation</a>
     */
    public final void updateOdometry() {
        m_odometry.update(gyro.getRotation2D(), frontLeftMotor.getEncoderPosition(),
                frontRightMotor.getEncoderPosition());
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return The current pose of the robot.
     */
    public final Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Gets the positions of the differential drive wheels.
     *
     * @return The positions of the differential drive wheels.
     */
    public final DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(frontLeftMotor.getEncoderVelocity(),
                frontRightMotor.getEncoderVelocity());
    }

    public final void setDriveVolts(double leftVolts, double rightVolts) {
        leftControllerGroup.setVoltage(leftVolts);
        rightControllerGroup.setVoltage(rightVolts);
        m_differentialDrive.feed();
    }

    public final double getAverageEncoderPosition() {
        return (frontLeftMotor.getEncoderPosition() + frontRightMotor.getEncoderPosition()) / 2.0;
    }

    public final void setMaxOutput(double maxOutput) {
        m_differentialDrive.setMaxOutput(maxOutput);
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
        frontLeftMotor.setSpeedMultiplier(fl);
        frontRightMotor.setSpeedMultiplier(fr);
        rearLeftMotor.setSpeedMultiplier(bl);
        rearRightMotor.setSpeedMultiplier(br);
    }

    /**
     * This method drives the robot using single-axis control.
     * 
     * @param xSpeed x-axis movement speed
     * @param y      y-axis movement speed
     * @param z      z-axis movement speed
     */
    public final void driveSingleAxis(double xSpeed, double y, double z) {
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
    public final void driveFieldOriented(double xSpeed, double ySpeed, double zRot, double gyroAngle) {
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
    public final void driveRobotOriented(double xSpeed, double ySpeed, double zRot) {
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
    public final void driveMecanum(double xSpeed, double ySpeed, double zRot) {
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
    public final boolean balanceOnStation() {
        double angle = gyro.getPitch();
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
}
