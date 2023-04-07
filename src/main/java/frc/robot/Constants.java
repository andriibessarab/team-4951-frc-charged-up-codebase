package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

public static final double BalanceKp = 0.5;
public static final double BalanceKi = 0.0;
public static final double BalanceKd = 0.05; 

  // Operator Input Constants (Joystick)
  public static final class OIConstants {
    public static final class DriverControl {
      public static final int kDriverControllerPort = 0;
      public static double kDriveDeadband = 0.2;
      public static double kRotationDeadband = 0.1;
      public static double kZeroCalibrateLeftY = 0.0;
      public static double kZeroCalibrateRightX = 0.0;
      public static double kZeroCalibrateLeftX = 0.0;
    }

    public static final class OperatorControl {
      public static int kOperatorControllerPort = 1;

      public static double kZeroCalibrateLeftY = 0.0;
      public static double kZeroCalibrateRightX = 0.0;
    }
  }

  public static final class DriveConstants {
    public static final int kFrontLeftMotorPort = 3;
    public static final int kRearLeftMotorPort = 4;
    public static final int kFrontRightMotorPort = 2;
    public static final int kRearRightMotorPort = 1;

    public static final boolean kFrontLeftInverted = false;
    public static final boolean kFrontRightInverted = true;
    public static final boolean kRearLeftInverted = false;
    public static final boolean kRearRightInverted = true;

    public static final double kTrackWidth = Units.inchesToMeters(23.0);;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.50);
    // Distance between centers of front and back wheels on robot

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Using tough box micros installed with the standard gear ratio of 8.45 : 1
    public static final double kGearRatio = 8.45;    // For every 8.45 encoder rotations, 1 wheel rotation
    public static final double kWheelRadiusMeters = Units.inchesToMeters(8.0 / 2.0); // #TODO 8" diameter wheels
    public static final double kRotationsToMeterConversionFactor = (1.0/kGearRatio) * 2.0 * Math.PI * kWheelRadiusMeters;
    public static final double kRpmToMeterPerSecondConversionFactor = kRotationsToMeterConversionFactor / 60.0;


    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // #TODO The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1, 0.8, 0.15);

    // #TODO Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

  public static final class AutoDriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ElevatorSubsystem {
    public static int kMotorPort = 5;
    public static int kSmartCurrentLimit = 20;  // #TODO 30A - is this sufficient to cause lift?
    public static double kDistancePerRevolution = Units.inchesToMeters(28.0/7.0); // measured 7 rotations = 28"
    public static double kVelocityMetersPerSecond = kDistancePerRevolution/60.0;

    public static double kMinHeight = 0.0;
    public static double kMaxHeight = 9.5; // value from smart dashboard

    public static class MOVE_UP {
      public static double kP = 1.0;
      public static double kI = 0.0;
      public static double kD = 0.0;
      public static double kIZone = 0.0;
      public static double kFeedForwardVelocity = 0.3149;   // Input to keep it from falling
    }
  
    public static class MOVE_DOWN {
      public static double kP = 0.0;
      public static double kI = 0.0;
      public static double kD = 0.0;
      public static double kIZone = 0.0;
      public static double kFeedForwardVelocity = 0.3149;
    }

    public static double kMaxControllerUpSpeed = 0.6;
    public static double kMaxControllerDownSpeed = -0.6;
    public static double kControllerDeadband = 0.1;

    // #TODO determine actual heights
    public static double kTopLayerHeight = 7.45;
    public static double kMidLayerHeight = 5.74;
    public static double kBottomLayerHeight = 0.1;

  }

  public static final class ArmSubsystem {
    public static int kMotorPort = 6;
    public static int kSmartCurrentLimit = 20;
    public static double kDistancePerRevolution = Units.inchesToMeters(28.0/7.0); // measured 7 rotations = 28"
    public static double kVelocityMetersPerSecond = kDistancePerRevolution/60.0;

    public static double kMinExtend = 0.0;
    public static double kMaxExtend = 11; // #TODO value from smart dashboard

    public static double kP = 0.1; // #TODO what should it be?
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kIZone = 0.0;
    public static double kFeedForwardVelocity = 0;

    public static double kMaxControllerUpSpeed = 0.6; // #TODO test those
    public static double kMaxControllerDownSpeed = -0.6;
    public static double kControllerDeadband = 0.1;
  }

  public static final class PivotSubsystem {
    public static int kMotorPort = 7;
    public static int kSmartCurrentLimit = 80;
    public static double kDistancePerRevolution = Units.inchesToMeters(28.0/7.0); // measured 7 rotations = 28"
    public static double kVelocityMetersPerSecond = kDistancePerRevolution/60.0;

    public static double kMinOut = 0.1;
    public static double kMaxOut = 3.5; // #TODO value from smart dashboard

    public static double kCloseValue = 3;
    public static double kOpenValue = 6; // #TODO value from smart dashboard

    public static double kP = 0.1; // #TODO what shouldi t be?
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kIZone = 0.0;
    public static double kFeedForwardVelocity = 0.5;

    public static double kMaxControllerUpSpeed = 0.5; // #TODO test those
    public static double kMaxControllerDownSpeed = -0.5;
    public static double kControllerDeadband = 0.1;
  }

  //Claw Stuff
  public static final class ClawSubsystemConstants {
    public static int forwardChannel = 15;
    public static int reverseChannel = 0;
    public static int kRMotorPort = 9;
    public static int kLMotorPort = 8;
  }

  // Limelight configuration: http://10.49.51.11:5801/
  public static final class LimelightSubsystem {
    public static String kLimelightName = "limelight";   // Default name, may change to support multiple limelight

    public static int kAprilTagsWatchPosePipelineID = 0;

    public static int kRetroTapePostAlignmentPipelineID = 1;

    public static int kAprilTagsPostAlignmentPipelineID = 2;
  }

  public static final class Vision {
    public static double kAlignmentKP = 0.05;   // P factor to use for auto alignment routines

    public static double kSpinClockwiseSearchRate = 0.05;  // Missing target spin rate for search during alignment
  }

  public static final class BalancePID {
    public static double kPosKP = 0.05;
    public static double kPosKI = 0.05;
    public static double kPosKD = 0.05;

    public static double kYawKP = 0.05;
    public static double kYawKI = 0.05;
    public static double kYawKD = 0.05;
  }
}
