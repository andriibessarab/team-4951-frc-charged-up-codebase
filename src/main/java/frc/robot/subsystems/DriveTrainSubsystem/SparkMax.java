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
public class SparkMax extends DriveSubsystem {
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
    public void setMotorSpeeds(double fl, double fr, double bl, double br) {
        frontLeftMotor.setSpeedMultiplier(fl);
        frontRightMotor.setSpeedMultiplier(fr);
        rearLeftMotor.setSpeedMultiplier(bl);
        rearRightMotor.setSpeedMultiplier(br);
    }
}
