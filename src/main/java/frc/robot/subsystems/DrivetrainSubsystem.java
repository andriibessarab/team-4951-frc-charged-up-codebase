package frc.robot.Subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DrivetrainConstants;

import java.util.function.Consumer;

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
public class DrivetrainSubsystem extends SubsystemBase {
    // Battery
    private PowerDistribution m_RobotPD;

    private ADIS16470_IMU m_gyro;

    // The motor objects used for controlling the robot's drivetrain
    private CANSparkMax m_FrontLeftMotor;
    private CANSparkMax m_FrontRightMotor;
    private CANSparkMax m_RearLeftMotor;
    private CANSparkMax m_RearRightMotor;

    // The encoder objects
    private RelativeEncoder m_FrontLeftEncoder;
    private RelativeEncoder m_FrontRightEncoder;
    private RelativeEncoder m_RearLeftEncoder;
    private RelativeEncoder m_RearRightEncoder;

    private MecanumDriveOdometry m_odometry;

    public DrivetrainSubsystem() {
        m_RobotPD = new PowerDistribution();

        m_gyro = new ADIS16470_IMU();

        m_FrontLeftMotor = new CANSparkMax(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_FrontRightMotor = new CANSparkMax(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        m_RearLeftMotor = new CANSparkMax(RobotMap.REAR_LEFT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_RearRightMotor = new CANSparkMax(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN, CANSparkMaxLowLevel.MotorType.kBrushless);

        m_FrontLeftEncoder = m_FrontLeftMotor.getEncoder();
        m_FrontRightEncoder = m_FrontRightMotor.getEncoder();
        m_RearLeftEncoder = m_RearLeftMotor.getEncoder();
        m_RearRightEncoder = m_RearRightMotor.getEncoder();

        resetEncoders();

        m_FrontLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        m_FrontRightEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        m_RearLeftEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);
        m_RearRightEncoder.setPositionConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor);

        m_FrontLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);
        m_FrontRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);
        m_RearLeftEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);
        m_RearRightEncoder.setVelocityConversionFactor(DrivetrainConstants.kLinearDistanceConversionFactor / 60);

        m_odometry = new MecanumDriveOdometry(DrivetrainConstants.kKinematics, getGyroAngle(), getWheelPositions());

        resetOdometry(new Pose2d());
    }

    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("Battery Voltage", m_RobotPD.getVoltage());

        SmartDashboard.putNumber("Front-Left Encoder Value", m_FrontLeftMotor.get());
        SmartDashboard.putNumber("Front-Right Encoder Value", m_FrontRightMotor.get());
        SmartDashboard.putNumber("Rear-Left Encoder Value", m_RearLeftMotor.get());
        SmartDashboard.putNumber("Rear-Right Encoder Value", m_RearRightMotor.get());

        SmartDashboard.putNumber("Front-Left Encoder Value", m_FrontLeftEncoder.getPosition());
        SmartDashboard.putNumber("Front-Right Encoder ValueE", m_FrontRightEncoder.getPosition());
        SmartDashboard.putNumber("Rear-Left Encoder Value", m_RearLeftEncoder.getPosition());
        SmartDashboard.putNumber("Rear-Right Encoder ValueE", m_RearRightEncoder.getPosition());
    }

    /**
     * Resets the encoder counts of a given motors to zero
     */
    public final void resetEncoders() {
        m_FrontLeftEncoder.setPosition(0);
        m_FrontRightEncoder.setPosition(0);
        m_RearLeftEncoder.setPosition(0);
        m_RearRightEncoder.setPosition(0);
    }

    /**
     * Resets the robot's odometry to a specified starting pose.
     * 
     * @param startingPose the pose to set the robot's odometry to
     */
    public final void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getGyroAngle(), getWheelPositions(), pose);
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
        m_odometry.update(getGyroAngle(), getWheelPositions());
    }

    /**
     * This method sets the speed for each motor on the robot.
     * 
     * @param fl speed for the front left motor
     * @param fr speed for the front right motor
     * @param bl speed for the back left motor
     * @param br speed for the back right motor
     */
    public final void setMotorSpeeds(double fl, double fr, double bl, double br) {
        m_FrontLeftMotor.set(fl * DrivetrainConstants.kMotorSpeedMultiplier);
        m_FrontRightMotor.set(fr * DrivetrainConstants.kMotorSpeedMultiplier);
        m_RearLeftMotor.set(bl * DrivetrainConstants.kMotorSpeedMultiplier);
        m_RearRightMotor.set(br * DrivetrainConstants.kMotorSpeedMultiplier);
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
    public boolean balanceOnStation() {
        double angle = m_gyro.getYComplementaryAngle();
        double error = -angle; // Negative because we want to balance on the opposite side of the gyro angle
        if (Math.abs(error) < DrivetrainConstants.kToleranceDegrees) { // If within tolerance, stop
            setMotorSpeeds(0, 0, 0, 0);
            return true;
        }

        double output = -Math.max(DrivetrainConstants.kMinOutput,
                Math.min(DrivetrainConstants.kMaxOutput, DrivetrainConstants.kBalancingController.calculate(angle)));
        setMotorSpeeds(output, output, output, output); // Set the motor speeds based on the PID output
        return false;

    }

    /**
     * Returns the angle of the yaw axis as a Rotation2d object.
     * 
     * @return the angle of the yaw axis as a Rotation2d object
     */
    public final Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    /**
     * Gets the positions of the mecanum drive wheels.
     *
     * @return The positions of the mecanum drive wheels.
     */
    public final MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
                m_FrontLeftEncoder.getPosition(),
                m_FrontRightEncoder.getPosition(),
                m_RearLeftEncoder.getPosition(),
                m_RearRightEncoder.getPosition());
    }

    /**
     * Returns robot's osition on the field
     * 
     * @return robot's position on the field
     */
    public final Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Gets the speed of the mecanum drive wheels.
     *
     * @return The speed of the mecanum drive wheels.
     */
    public Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds() {
        return mecanumDriveWheelSpeeds -> {
            m_FrontLeftMotor.set(
                    (mecanumDriveWheelSpeeds.frontLeftMetersPerSecond)
                            / (DrivetrainConstants.kWheelDiameterMeters * DrivetrainConstants.kGearRatio
                                    * DrivetrainConstants.kRadiansePerSecondToRPMConverisionFactor));
            m_FrontRightMotor.set(
                    (mecanumDriveWheelSpeeds.frontRightMetersPerSecond)
                            / (DrivetrainConstants.kWheelDiameterMeters * DrivetrainConstants.kGearRatio
                                    * DrivetrainConstants.kRadiansePerSecondToRPMConverisionFactor));
            m_RearLeftMotor.set(
                    (mecanumDriveWheelSpeeds.rearLeftMetersPerSecond)
                            / (DrivetrainConstants.kWheelDiameterMeters * DrivetrainConstants.kGearRatio
                                    * DrivetrainConstants.kRadiansePerSecondToRPMConverisionFactor));
            m_RearRightMotor.set(
                    (mecanumDriveWheelSpeeds.rearRightMetersPerSecond)
                            / (DrivetrainConstants.kWheelDiameterMeters * DrivetrainConstants.kGearRatio
                                    * DrivetrainConstants.kRadiansePerSecondToRPMConverisionFactor));
        };
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPMecanumControllerCommand(
                        traj,
                        this::getPose,
                        DrivetrainConstants.kKinematics,
                        DrivetrainConstants.kXController,
                        DrivetrainConstants.kYController,
                        DrivetrainConstants.kZController,
                        DrivetrainConstants.kMaxWheelVelocityMetersPerSecond,
                        outputWheelSpeeds(),
                        true,
                        this));
    }

}
