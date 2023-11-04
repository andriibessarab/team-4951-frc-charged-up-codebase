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
    public static final class OIConstants {
        // Controller for driving
        public static final class DriverControl {
            public static final int kDriverControllerPort = 0;
            public static final double kDriveDeadband = 0.2;
            public static final double kRotationDeadband = 0.1;
            public static final double kZeroCalibrateLeftY = 0.0;
            public static final double kZeroCalibrateRightX = 0.0;
            public static final double kZeroCalibrateLeftX = 0.0;
        }

        // Controller for operating intake
        public static final class OperatorControl {
            public static final int kOperatorControllerPort = 1;
            public static final double kZeroCalibrateLeftY = 0.0;
            public static final double kZeroCalibrateRightX = 0.0;
        }
    }

    public static final class DriveConstants {
        // Motor ports
        public static final int kRearRightMotorPort = 1;
        public static final int kFrontRightMotorPort = 2;
        public static final int kFrontLeftMotorPort = 3;
        public static final int kRearLeftMotorPort = 4;

        // Motor inversions
        public static final boolean kFrontLeftInverted = false;
        public static final boolean kFrontRightInverted = true;
        public static final boolean kRearLeftInverted = false;
        public static final boolean kRearRightInverted = true;

        // Misc motor constants
        public static final int kSmartCurrentLimitAmps = 40;

        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(23.0);;
        // Distance between centers of front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(20.50);

        // Kinematics
        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Using tough box micros installed with the standard gear ratio of 8.45 : 1
        public static final double kGearRatio = 8.45; // For every 8.45 encoder rotations, 1 wheel rotation
        public static final double kWheelRadiusMeters = Units.inchesToMeters(8.0 / 2.0); // 8" diameter wheels
        public static final double kRotationsToMeterConversionFactor = (1.0 / kGearRatio) * 2.0 * Math.PI * kWheelRadiusMeters;
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

    public static final class ElevatorSubsystemConstants {
        // Motor constants
        public static final int kMotorPort = 5;
        public static final int kSmartCurrentLimit = 20;

        // Encoder constants
        public static final double kDistancePerRevolution = Units.inchesToMeters(28.0/7.0); // measured 7 rotations = 28"
        public static final double kVelocityMetersPerSecond = kDistancePerRevolution/60.0;
    
        // Soft limit constants
        public static final double kMinHeight = 0.1;
        public static final double kMaxHeight = 9.5;
    
        // PID controller constants
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIZone = 0.0;
        public static final double kFeedForwardVelocity = 0.3149; // input to keep it from falling
    
        // Controller constants
        public static final double kMaxControllerUpSpeed = 0.6;
        public static final double kMaxControllerDownSpeed = -0.6;
        public static final double kControllerDeadband = 0.1;
    
        // Field heights constants
        public static final double kTopLayerHeight = 7.45;
        public static final double kMidLayerHeight = 5.74;
        public static final double kBottomLayerHeight = 0.1;

    }

    public static final class ArmSubsystemConstants {
        // Motor constants
        public static final int kMotorPort = 6;
        public static final int kSmartCurrentLimit = 20;

        // Encoder constants
        public static final double kDistancePerRevolution = Units.inchesToMeters(28.0/7.0);
        public static final double kVelocityMetersPerSecond = kDistancePerRevolution/60.0;

        // Soft limit constants
        public static final double kMinExtend = 0.0;
        public static final double kMaxExtend = 11;
    
        // PID controller constants
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIZone = 0.0;
        public static final double kFeedForwardVelocity = 0;
    
        // Controller constants
        public static final double kMaxControllerUpSpeed = 0.6;
        public static final double kMaxControllerDownSpeed = -0.6;
        public static final double kControllerDeadband = 0.1;
    }
    
    public static final class PivotSubsystem {
        // Motor constants
        public static final int kMotorPort = 7;
        public static final int kSmartCurrentLimit = 80;

        // Encoder constants
        public static final double kDistancePerRevolution = Units.inchesToMeters(28.0/7.0);
        public static final double kVelocityMetersPerSecond = kDistancePerRevolution/60.0;
    
        // Soft limit constants
        public static final double kMinOut = 0.1;
        public static final double kMaxOut = 3.5;
    
        // PID controller constants
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIZone = 0.0;
        public static final double kFeedForwardVelocity = 0.5;
    
        // Controller constants
        public static final double kMaxControllerUpSpeed = 0.5;
        public static final double kMaxControllerDownSpeed = -0.5;
        public static final double kControllerDeadband = 0.1;
  }

    public static final class ClawSubsystemConstants {
    public static final int kForwardChannel = 15;
    public static final int kReverseChannel = 0;
    public static final int kRMotorPort = 9;
    public static final int kLMotorPort = 8;
  }

    public static final class LimelightSubsystem {
        // Limelight configuration: http://10.49.51.11:5801/

        // Limelight names
        public static String kLimelightName = "limelight"; // Default name, may change to support multiple limelight

        // Pipelines
        public static int kAprilTagsWatchPosePipelineID = 0;
        public static int kRetroTapePostAlignmentPipelineID = 1;
        public static int kAprilTagsPostAlignmentPipelineID = 2;
    }

    public static final class Vision {
        public static double kAlignmentKP = 0.05; // P factor to use for auto alignment routines
        public static double kSpinClockwiseSearchRate = 0.05; // Missing target spin rate for search during alignment
    }

    public static final class BalancePID {
        public static final double BalanceKp = 0.5;
        public static final double BalanceKi = 0.0;
        public static final double BalanceKd = 0.05;
    }

    public static final class ScoringConstants {
        public static final double kElevatorTopLevel = 7.45;
        public static final double kElevatorMidLevel = 5.74;
        public static final double kElevatorBottomLevel = 0.0; // previously was 0.1

        public static final double kPivotIn = 0.1;
        public static final double kPivotOut = 2.3;

        public static final double kOuttakeSpeedTopLevel = 0.6;
        public static final double kouttakeSpeedrMidLevel = 0.2;
        public static final double kOuttakeSpeedManual = 0.5;
    }
}