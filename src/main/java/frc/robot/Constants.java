package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * This class defines constants used for this robot's subsystems.
 */
public class Constants {
    /**
     * This class defines constants that are used in drivetrain.
     */
    public static final class DrivetrainConstants {
        public static final int kMotorSpeedMultiplier = 1;

        /*
         * Locations of wheel relative to the cneter of robot(in meters)
         * Positive x value means closer to the front of the robot
         * Positive y value means closer to the left of the robot
         */
        public static final Translation2d frontLeftWheelLocation = new Translation2d(12, 11);
        public static final Translation2d frontRightWheelLocation = new Translation2d(12, -11);
        public static final Translation2d rearLeftWheelLocation = new Translation2d(-12, 11);
        public static final Translation2d rearRightWheelLocation = new Translation2d(-12, -11);

        public static final PIDController kXController = new PIDController(0.1, 0.05, 0);
        public static final PIDController kYController = new PIDController(0.1, 0.05, 0);
        public static final PIDController kZController = new PIDController(0.3, 0.1, 0.01);

        public static final double kMaxWheelVelocityMetersPerSecond = 3.0; // Max wheel velocity meters per second

        // need to obtain them through sysid
        public static double ksVolts;
        public static double kvVoltSecondsPerMinuite;
        public static double kaVoltSecondsSquaredPerMinuite;
        public static double kpDriveVel;

        public static final double kTrackWidthMeters = Units.inchesToMeters(20.5); // horizontal distance between two
                                                                                   // wheels
        public static final MecanumDriveKinematics kKinematics = new MecanumDriveKinematics(
                frontLeftWheelLocation, frontRightWheelLocation, rearLeftWheelLocation, rearRightWheelLocation);

        // Values obtained thrrough WILIB documentation
        public static final double kMaxSpeedMeterserSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 23;
        public static final double kRamseteZeta = 0.7;

        public static final double kRadiansePerSecondToRPMConverisionFactor = 0.10472;
        public static final double kGearRatio = 10.71;
        public static final double kWheelRadiusInches = 2.75;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelRadiusInches*2);
        public static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10));

        public static final double kProportionalGain = 0.03; // Proportional gain
        public static final double kIntegralGain = 0.00; // Integral gain
        public static final double kDerivativeGain = 0.00; // Derivative gain
        public static final double kToleranceDegrees = 4.0f; // Tolerance for gyro angle
        public static final double kMaxOutput = 0.5; // Maximum output
        public static final double kMinOutput = -0.5; // Minimum output

        public final static PIDController kBalancingController = new PIDController(
                DrivetrainConstants.kProportionalGain,
                DrivetrainConstants.kIntegralGain, DrivetrainConstants.kDerivativeGain);
    }

    /**
     * This class defines constants that are used for limelight vision.
     */
    public final class LimelightConstants {
        public final static int kReflectiveTapePipeline = 1;
        public final static int kAprilTagsPipeline = 0;

        // Approximate(need to be measured)
        public final static int MAX_VERT_OFFSET_FOR_LOW = 10;
        public final static int HEIGHT_TO_LOW = 24;
        public final static int MAX_VERT_OFFSET_FOR_MED = 20;
        public final static int HEIGHT_TO_MED = 36;
        public final static int MAX_VERT_OFFSET_FOR_HIGH = 30;
        public final static int HEIGHT_TO_HIGH = 48;
    }
}
