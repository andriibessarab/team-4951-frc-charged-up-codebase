package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import static frc.robot.Constants.DriveConstants.*;
/**
 * Uses MecanumDrive for movement:
 *
 * Which uses the NWU axes convention (North-West-Up as external reference in the world
 * frame). The positive X axis points ahead, the positive Y axis points to the left, and the
 * positive Z axis points up. Rotations follow the right-hand rule, so counterclockwise rotation
 * around the Z axis is positive.
 *
 * MecanumDrive also enables MotorSafety by default.
 */
public class Subsys_Drive extends SubsystemBase {
  // Motors
  private final CANSparkMax m_frontLeft = new CANSparkMax(kFrontLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(kRearLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(kFrontRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(kRearRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

  // Gyro
  private final PigeonIMU m_gyro = new PigeonIMU(40);

  // Mecanum drive
  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // Odometry class for tracking robot pose
  private MecanumDriveOdometry m_odometry;

  public final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public Subsys_Drive() {
    // Reset motors to factory defaults
    m_frontLeft.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();

    m_frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rearLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rearRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_frontLeft.setSmartCurrentLimit(48);
    m_rearLeft.setSmartCurrentLimit(48);
    m_frontRight.setSmartCurrentLimit(48);
    m_rearRight.setSmartCurrentLimit(48);

    // Set the distance per pulse for the encoders
    m_frontLeftEncoder.setPositionConversionFactor(kRotationsToMeterConversionFactor);
    m_rearLeftEncoder.setPositionConversionFactor(kRotationsToMeterConversionFactor);
    m_frontRightEncoder.setPositionConversionFactor(kRotationsToMeterConversionFactor);
    m_rearRightEncoder.setPositionConversionFactor(kRotationsToMeterConversionFactor);

    m_frontLeftEncoder.setVelocityConversionFactor(kRpmToMeterPerSecondConversionFactor);
    m_rearLeftEncoder.setVelocityConversionFactor(kRpmToMeterPerSecondConversionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(kRpmToMeterPerSecondConversionFactor);
    m_rearRightEncoder.setVelocityConversionFactor(kRpmToMeterPerSecondConversionFactor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(kFrontLeftInverted);
    m_rearLeft.setInverted(kRearLeftInverted);
    m_frontRight.setInverted(kFrontRightInverted);
    m_rearRight.setInverted(kRearRightInverted);

    SmartDashboard.putData(m_field);

    m_odometry = new MecanumDriveOdometry(
            kDriveKinematics,
            getGyroRotation2d(),
            new MecanumDriveWheelPositions());
    
            m_drive.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(getGyroRotation2d(), getCurrentWheelDistances());
    m_field.setRobotPose(getPose());
    updateSmartDashboard();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getGyroRotation2d(), getCurrentWheelDistances(), pose);
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param xSpeed Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(xSpeed, ySpeed, rot, getGyroRotation2d());
    } else {
      m_drive.driveCartesian(xSpeed, ySpeed, rot);
    }
  }

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

    if(leftFront==0&&leftRear==0&&rightFront==0&&rightRear==0){
      this.m_frontLeft.stopMotor();
      this.m_rearLeft.stopMotor();
      this.m_rearRight.stopMotor();
      this.m_frontRight.stopMotor();
    } else{
      m_frontLeft.set(leftFront);
      m_rearLeft.set(leftRear);
      m_rearRight.set(rightRear);
      m_frontRight.set(rightFront);
    }
  }

  /**
   * Sets the drive MotorController to a voltage.
   */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public RelativeEncoder getFrontLeftEncoder() {
    return m_frontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public RelativeEncoder getRearLeftEncoder() {
    return m_rearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public RelativeEncoder getFrontRightEncoder() {
    return m_frontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public RelativeEncoder getRearRightEncoder() {
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getVelocity(),
        m_rearLeftEncoder.getVelocity(),
        m_frontRightEncoder.getVelocity(),
        m_rearRightEncoder.getVelocity());
  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_rearRightEncoder.getPosition());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public Rotation2d getGyroRotation2d() {
    // https://www.chiefdelphi.com/t/1-does-legacy-navx-work-today-and-2-why-is-adis16470-imu-missing-getrotation2d/420343
    // https://github.com/wpilibsuite/allwpilib/issues/4876
    return Rotation2d.fromDegrees(getPitch());
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }
  
  public double getRoll() {
      return m_gyro.getRoll();
  }

  public double getYaw() {
    return m_gyro.getYaw();
  }

  // returns the magnititude of the robot's tilt calculated by the root of
  // pitch^2 + roll^2, used to compensate for diagonally mounted rio
  public double getTilt() {
      double pitch = getPitch();
      double roll = getRoll();
      if ((pitch + roll) >= 0) {
          return Math.sqrt(pitch * pitch + roll * roll);
      } else {
          return -Math.sqrt(pitch * pitch + roll * roll);
      }
    }
    
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation2d().getDegrees();
  }

  // https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#ppmecanumcontrollercommand
  public void setCurrentWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
      m_frontLeft.set(speeds.frontLeftMetersPerSecond);
      m_rearLeft.set(speeds.rearLeftMetersPerSecond);
      m_frontRight.set(speeds.frontRightMetersPerSecond);
      m_rearRight.set(speeds.rearRightMetersPerSecond);
  }

  public void updateSmartDashboard() {
    PigeonState gyroState = m_gyro.getState();
    switch (gyroState) {
      case NoComm:
        SmartDashboard.putString("drive/gyro/state", "No Comm");
        break;
      case Initializing:
        SmartDashboard.putString("drive/gyro/state", "Init");
        break;
      case Ready:
        SmartDashboard.putString("drive/gyro/state", "Ready");
        break;
      case UserCalibration:
        SmartDashboard.putString("drive/gyro/state", "User Calib");
        break;
      case Unknown:
        SmartDashboard.putString("drive/gyro/state", "Unknown");
        break;
      default:
        SmartDashboard.putString("drive/gyro/state", "Really Unknown");

    }
    SmartDashboard.putNumber("drive/gyro/yaw [+]", getYaw());
    SmartDashboard.putNumber("drive/gyro/pitch [-90, 90]", getPitch());
    SmartDashboard.putNumber("drive/gyro/roll [-90, 90]", getRoll());
  }
}