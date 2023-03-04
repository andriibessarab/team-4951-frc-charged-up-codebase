package frc.robot.subsystems.DrivetrainSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotMap;


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
public class SparkMaxDrivetrainSubsystem extends DrivetrainSubsystemBase {
    // The motor objects used for controlling the robot's drivetrain
    public CANSparkMax rearLeftMotor;
    public CANSparkMax rearRightMotor;
    public CANSparkMax frontLeftMotor;
    public CANSparkMax frontRightMotor;

    public RelativeEncoder rearLeftEncoder;
    public RelativeEncoder rearRightEncoder;
    public RelativeEncoder frontLeftEncoder;
    public RelativeEncoder frontRightEncoder;

    //private MotorControllerGroup leftControllerGroup;
    //private MotorControllerGroup rightControllerGroup;

   // private DifferentialDrive m_differentialDrive;
    //private DifferentialDriveOdometry m_odometry;

    /**
     * Constructor for the DrivetrainSubsystem class.
     *
     */
    public SparkMaxDrivetrainSubsystem() {
        super();

        rearLeftMotor = new CANSparkMax(RobotMap.REAR_LEFT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeftMotor = new CANSparkMax(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);

        rearLeftEncoder = rearLeftMotor.getEncoder();
        rearRightEncoder = rearRightMotor.getEncoder();
        frontLeftEncoder = frontLeftMotor.getEncoder();
        frontRightEncoder = frontRightMotor.getEncoder();
    
        /* 
        m_odometry = new DifferentialDriveOdometry(gyro.getRotation2D(), frontLeftMotor.getEncoderPosition(),
                frontRightMotor.getEncoderPosition());

        leftControllerGroup = new MotorControllerGroup(frontLeftMotor.getMotorInstance(),
                rearLeftMotor.getMotorInstance());
        rightControllerGroup = new MotorControllerGroup(frontRightMotor.getMotorInstance(),
                rearRightMotor.getMotorInstance());

        m_differentialDrive = new DifferentialDrive(leftControllerGroup, rightControllerGroup);
        */

        resetEncoders();

        frontLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        frontRightEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        rearLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        rearRightEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);

        frontLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor/60);
        frontRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor/60);
        rearLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor/60);
        rearRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor/60);

        // resetOdometry(new Pose2d());
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Left-encoder value(meters)", frontLeftEncoder.getPosition());
        SmartDashboard.putNumber("Right-encoder value(meters)", frontRightEncoder.getPosition());
    }

    /**
     * Resets the encoder counts of a given motors to zero
     */
    public final void resetEncoders() {
        frontLeftEncoder.setPosition(0);
        frontRightEncoder.setPosition(0);
        rearLeftEncoder.setPosition(0);
        rearRightEncoder.setPosition(0);
    }

    /**
     * Resets the robot's odometry to a specified starting pose.
     * 
     * @param startingPose the pose to set the robot's odometry to
     *
    public final void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(gyro.getRotation2D(), frontLeftMotor.getEncoderPosition(),
                frontRightMotor.getEncoderPosition(), pose);
    }
    */

    /**
     * Updates the robot's odometry information using the current sensor readings.
     * This method calculates the robot's position and orientation based on readings
     * from the
     * gyro and the mecanum drive motor encoders.
     * 
     * @see <a href=
     *      "https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/odometry.html">WPILib
     *      Odometry documentation</a>
     *
    public final void updateOdometry() {
        m_odometry.update(gyro.getRotation2D(), frontLeftMotor.getEncoderPosition(),
                frontRightMotor.getEncoderPosition());
    }
*/
    /**
     * Gets the current pose of the robot.
     *
     * @return The current pose of the robot.
     *
    public final Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
*/
    /**
     * Gets the positions of the differential drive wheels.
     *
     * @return The positions of the differential drive wheels.
     *
    public final DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(frontLeftMotor.getEncoderVelocity(),
                frontRightMotor.getEncoderVelocity());
    }
*/

    /* 
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
    */

    /**
     * This method sets the speed for each motor on the robot.
     * 
     * @param fl speed for the front left motor
     * @param fr speed for the front right motor
     * @param bl speed for the back left motor
     * @param br speed for the back right motor
     */
    @Override
    public final void setMotorSpeeds(double fl, double fr, double bl, double br) {
        frontLeftMotor.set(fl * RobotMap.MOTOR_SPEED_MULTIPLIER);
        frontRightMotor.set(fr * RobotMap.MOTOR_SPEED_MULTIPLIER);
        rearLeftMotor.set(bl * RobotMap.MOTOR_SPEED_MULTIPLIER);
        rearRightMotor.set(br * RobotMap.MOTOR_SPEED_MULTIPLIER);
    }
}
