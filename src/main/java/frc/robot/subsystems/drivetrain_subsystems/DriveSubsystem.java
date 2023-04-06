package frc.robot.subsystems.drivetrain_subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

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
public class DriveSubsystem extends SubsystemBase {
  // Motors
  private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_frontLeftEncoder = m_frontLeft.getEncoder();
  private final RelativeEncoder m_rearLeftEncoder = m_rearLeft.getEncoder();
  private final RelativeEncoder m_frontRightEncoder = m_frontRight.getEncoder();
  private final RelativeEncoder m_rearRightEncoder = m_rearRight.getEncoder();

  // Gyro
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  // private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final PigeonIMU m_gyro = new PigeonIMU(40);
  private long m_gyroLastResetTimeMS = 0;
  private double m_gyroYawOffset = 0.0;
  private double m_gyroPitchOffset = 0.0;
  private double m_gyroRollOffset = 0.0;

  // Mecanum drive
  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

  // Odometry class for tracking robot pose
  private MecanumDriveOdometry m_odometry;

  
  private final Sendable gyroYawSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("gyro/yaw", this::getYaw, this::setYaw);
    }

    public double getYaw() {
        return m_gyro.getYaw();
    }

    public void setYaw(double x) {

    }
  };

  private final Sendable gyroPitchSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("gyro/pitch", this::getPitch, this::setPitch);
    }

    public double getPitch() {
        return m_gyro.getPitch();
    }

    public void setPitch(double x) {

    }
  };

  private final Sendable gyroRollSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("gyro/roll", this::getRoll, this::setRoll);
    }

    public double getRoll() {
        return m_gyro.getRoll();
    }

    public void setRoll(double x) {

    }
  };


  private final Sendable gyroSendable = new Sendable() {
      double[] yawPitchRoll = new double[3];

      @Override
      public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("Accelerometer");
          builder.addDoubleProperty("X", this::getX, this::setX);
          builder.addDoubleProperty("Y", this::getY, this::setY);
          builder.addDoubleProperty("Z", this::getZ, this::setZ);
      }

      public double getX() {
          m_gyro.getYawPitchRoll(yawPitchRoll);
          return yawPitchRoll[0];
      }

      public double getY() {
        m_gyro.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll[1];
      }

      public double getZ() {
        m_gyro.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll[2];
      }

      public void setX(double x) {}

      public void setY(double y) {}

      public void setZ(double z) {}
  };

  private final Sendable mecanumDrivSendable = new Sendable() {
    double[] yawPitchRoll = new double[3];

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Accelerometer");
        builder.addDoubleProperty("X", this::getX, this::setX);
        builder.addDoubleProperty("Y", this::getY, this::setY);
        builder.addDoubleProperty("Z", this::getZ, this::setZ);
    }

    public double getX() {
        m_gyro.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll[0];
    }

    public double getY() {
      m_gyro.getYawPitchRoll(yawPitchRoll);
      return yawPitchRoll[1];
    }

    public double getZ() {
      m_gyro.getYawPitchRoll(yawPitchRoll);
      return yawPitchRoll[2];
    }

    public void setX(double x) {}

    public void setY(double y) {}

    public void setZ(double z) {}
};

  // Shufflebored tab & widgets
  ShuffleboardTab m_driveShuffleboard;

  public final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Reset motors to factory defaults
    m_frontLeft.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();

    m_frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rearLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rearRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_frontLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimitAmps);
    m_rearLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimitAmps);
    m_frontRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimitAmps);
    m_rearRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimitAmps);

    // Set the distance per pulse for the encoders
    m_frontLeftEncoder.setPositionConversionFactor(DriveConstants.kRotationsToMeterConversionFactor);
    m_rearLeftEncoder.setPositionConversionFactor(DriveConstants.kRotationsToMeterConversionFactor);
    m_frontRightEncoder.setPositionConversionFactor(DriveConstants.kRotationsToMeterConversionFactor);
    m_rearRightEncoder.setPositionConversionFactor(DriveConstants.kRotationsToMeterConversionFactor);

    m_frontLeftEncoder.setVelocityConversionFactor(DriveConstants.kRpmToMeterPerSecondConversionFactor);
    m_rearLeftEncoder.setVelocityConversionFactor(DriveConstants.kRpmToMeterPerSecondConversionFactor);
    m_frontRightEncoder.setVelocityConversionFactor(DriveConstants.kRpmToMeterPerSecondConversionFactor);
    m_rearRightEncoder.setVelocityConversionFactor(DriveConstants.kRpmToMeterPerSecondConversionFactor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontLeft.setInverted(DriveConstants.kFrontLeftInverted);
    m_rearLeft.setInverted(DriveConstants.kRearLeftInverted);
    m_frontRight.setInverted(DriveConstants.kFrontRightInverted);
    m_rearRight.setInverted(DriveConstants.kRearRightInverted);

    SmartDashboard.putData(m_field);

    resetEncoders();

    m_odometry = new MecanumDriveOdometry(
            DriveConstants.kDriveKinematics,
            getGyroRotation2d(),
            new MecanumDriveWheelPositions());
    
    m_drive.setSafetyEnabled(false);

    m_driveShuffleboard = Shuffleboard.getTab("Drive");

    m_driveShuffleboard.add("Gyro Yaw", gyroYawSendable)
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(2, 2)
    .withPosition(0, 0);

    m_driveShuffleboard.add("Gyro Pitch", gyroPitchSendable)
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(2, 2)
    .withPosition(3, 0);

    m_driveShuffleboard.add("Gyro Roll", gyroRollSendable)
    .withWidget(BuiltInWidgets.kGyro)
    .withSize(2, 2)
    .withPosition(5, 0);
}



  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(getGyroRotation2d(), getCurrentWheelDistances());
    m_field.setRobotPose(getPose());

    // Smart dashboard
    updateSmartDashboardGyro(m_gyro);

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

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyroLastResetTimeMS = System.currentTimeMillis();

    // Calling gyro reset SHOULD re-zero (https://wiki.analog.com/first/adis16470_imu_frc)
    // the IMU but often after calling reset it seems to lock up the IMU and the getAngle()
    // never changes. So rather the having it reset, we can just track our own offsets.
    m_gyroYawOffset = 0.0;
    m_gyroPitchOffset = 0.0;
    m_gyroRollOffset = 0.0;

    // Use normal functions, so clear the offsets above first.
    m_gyroYawOffset = getTilt();
    m_gyroPitchOffset = getPitch();
    m_gyroRollOffset = getRoll();
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }

  public double getTilt() {
    return m_gyro.getYaw() % 360;
  }

  // public void ResetHeading() {
  //   return m_gyro.reset()
  // }

  public Rotation2d getGyroRotation2d() {
    // https://www.chiefdelphi.com/t/1-does-legacy-navx-work-today-and-2-why-is-adis16470-imu-missing-getrotation2d/420343
    // https://github.com/wpilibsuite/allwpilib/issues/4876
    return Rotation2d.fromDegrees(getPitch());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation2d().getDegrees();
  }

//    double yaw = getYaw();  // Range [-360, 360]
//    if (Math.abs(yaw) <= 180.0) {
//      return yaw;
//    } else {
//      // Need to provide heading in range [-180, 180]
//      if (yaw < 0.0) {
//        return yaw + 360.0;   // ie. -190 becomes 170, -350 becomes 10
//      } else {
//        return yaw - 360.0;   // ie. 190 becomes -170, 350 becomes -10
//      }
//    }
//  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return 0;
  }

  // https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#ppmecanumcontrollercommand
  public void setCurrentWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
      m_frontLeft.set(speeds.frontLeftMetersPerSecond);
      m_rearLeft.set(speeds.rearLeftMetersPerSecond);
      m_frontRight.set(speeds.frontRightMetersPerSecond);
      m_rearRight.set(speeds.rearRightMetersPerSecond);
  }

  /////////////////////////////
  //     SMART DASHBOARD     //
  /////////////////////////////

  public void updateSmartDashboard() {
    /*
    SmartDashboard.putNumber("drive/gyro/seconds since reset", (System.currentTimeMillis()- m_gyroLastResetTimeMS)/1000.0);
    SmartDashboard.putNumber("drive/gyro/rate [degrees per sec]", getTurnRate());
    SmartDashboard.putNumber("drive/gyro/heading [-180,180]", getHeading());
    SmartDashboard.putNumber("drive/gyro/yaw [-360,360]", getYaw());
    SmartDashboard.putNumber("drive/gyro/pitch [-360,360]", getPitch());
    SmartDashboard.putNumber("drive/gyro/roll [-360,360]", getRoll());
     */

    var wheelPositions = getCurrentWheelDistances();
    SmartDashboard.putNumber("drive/front left/position [m]", wheelPositions.frontLeftMeters);
    SmartDashboard.putNumber("drive/front right/position [m]", wheelPositions.frontRightMeters);
    SmartDashboard.putNumber("drive/rear left/position [m]", wheelPositions.rearLeftMeters);
    SmartDashboard.putNumber("drive/rear right/position [m]", wheelPositions.rearRightMeters);

    var wheelSpeeds = getCurrentWheelSpeeds();
    SmartDashboard.putNumber("drive/front left/speed [m|s]", wheelSpeeds.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("drive/front right/speed [m|s]", wheelSpeeds.frontRightMetersPerSecond);
    SmartDashboard.putNumber("drive/rear left/speed [m|s]", wheelSpeeds.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("drive/rear right/speed [m|s]", wheelSpeeds.rearRightMetersPerSecond);

    var pose = m_odometry.getPoseMeters();
    var poseRotation2d = pose.getRotation();
    var poseTranslation2d = pose.getTranslation();
    SmartDashboard.putNumber("drive/odometry/x", pose.getX());
    SmartDashboard.putNumber("drive/odometry/y", pose.getY());
    SmartDashboard.putNumber("drive/odometry/degrees", poseRotation2d.getDegrees());
    SmartDashboard.putNumber("drive/odometry/translation/x", poseTranslation2d.getX());
    SmartDashboard.putNumber("drive/odometry/translation/y", poseTranslation2d.getY());
    SmartDashboard.putNumber("drive/odometry/translation/degrees", poseTranslation2d.getAngle().getDegrees());

    String names[] = { "front left", "front right", "rear left", "rear right" };
    RelativeEncoder encoders[] = { m_frontLeftEncoder, m_frontRightEncoder, m_rearLeftEncoder, m_rearRightEncoder };
    CANSparkMax motors[] = { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };
    for ( int index = 0; index < names.length; index++ ) {
       updateSmartDashboardMotor(names[index], motors[index]);
       updateSmartDashboardEncoder(names[index], encoders[index]);
    }
  }

  void updateSmartDashboardMotor(String name, CANSparkMax motor) {
    SmartDashboard.putNumber("drive/"+name+"/motor/faults", motor.getFaults());
    SmartDashboard.putNumber("drive/"+name+"/motor/temperature", motor.getMotorTemperature());
    SmartDashboard.putNumber("drive/"+name+"/motor/applied output", motor.getAppliedOutput());
    SmartDashboard.putNumber("drive/"+name+"/motor/output current", motor.getOutputCurrent());
    SmartDashboard.putNumber("drive/"+name+"/motor/set speed", motor.get());
    SmartDashboard.putNumber("drive/"+name+"/motor/voltage compensated", motor.getVoltageCompensationNominalVoltage());
  }

  void updateSmartDashboardEncoder(String name, RelativeEncoder encoder) {
    SmartDashboard.putNumber("drive/"+name+"/encoder/position", encoder.getPosition());
    SmartDashboard.putNumber("drive/"+name+"/encoder/velocity", encoder.getVelocity());
    SmartDashboard.putNumber("drive/"+name+"/encoder/counts per revolution", encoder.getCountsPerRevolution());
    SmartDashboard.putNumber("drive/"+name+"/encoder/position conversion factor", encoder.getPositionConversionFactor());
    SmartDashboard.putNumber("drive/"+name+"/encoder/velocity conversion factor", encoder.getVelocityConversionFactor());
  }

  void updateSmartDashboardGyro(PigeonIMU gyro) {
    PigeonState state = m_gyro.getState();

  // Yaw: Continuous angle (Degrees) of the robot’s heading. Value will overflow at ±23040 degrees.
  SmartDashboard.putNumber("gyro/yaw", gyro.getYaw() % 360);
  
  // Pitch: (Degrees: -90 to 90)
  SmartDashboard.putNumber("gyro/pitch", gyro.getPitch());

  // Roll: (Degrees: -90 to 90)
  SmartDashboard.putNumber("gyro/roll", gyro.getRoll());

  // Compass Heading (Degrees) and Magnitude (µT)
  SmartDashboard.putNumber("gyro/compassHeading", gyro.getCompassHeading());

  // - Fused Heading (Degrees) and Fusion Status (Booleans)
  SmartDashboard.putNumber("gyro/fusedHeading", gyro.getFusedHeading());

  // Temperature (ºC)
  SmartDashboard.putNumber("gyro/temperature", gyro.getTemp());

  // Uptime (ms): How long the Pigeon has been powered (Capped at 255 seconds).
  SmartDashboard.putNumber("gyro/uptime", gyro.getUpTime());

  switch(state) {
    case NoComm:
      SmartDashboard.putString("gyro/state", "no comm");
      break;
    case Initializing:
      SmartDashboard.putString("gyro/state", "initializing");
      break;
    case Ready:
      SmartDashboard.putString("gyro/state", "ready");
      break;
    case UserCalibration:
      SmartDashboard.putString("gyro/state", "user calibrated");
      break;
    case Unknown:
      SmartDashboard.putString("gyro/state", "unknown");
      break;
    default:
      SmartDashboard.putString("gyro/state", "really unknown");
  }

  // UNPRINTED DATA
  // - Pigeon State: Used to determine when Pigeon is finished initializing, and that Pigeon is connected.
  // - Quaternion (6D)
  // - Accelerometer Angles (tilt angles in degrees between two axis, XZ, YZ, XY respectively)
  // - Magnetometer Axes (µT)
  // - Gyro Angles (Degrees) (accumulated using gyroscope only)
  }

}
