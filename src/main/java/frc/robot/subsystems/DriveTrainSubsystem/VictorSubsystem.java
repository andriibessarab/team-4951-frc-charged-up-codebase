package frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.RobotMap;

public class VictorSubsystem extends DriveSubsystem{
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
    
    // The motor objects used for controlling the robot's drivetrain
    public Victor rearLeftMotor;
    public Victor rearRightMotor;
    public Victor frontLeftMotor;
    public Victor frontRightMotor;

    public VictorSubsystem(){
        this.frontLeftMotor = new Victor(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN);
        this.frontRightMotor = new Victor(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN);
        this.rearLeftMotor = new Victor(RobotMap.REAR_LEFT_MOTOR_PWM_PIN);
        this.rearRightMotor = new Victor(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN);
    }

    public void setMotorSpeeds(double fl, double fr, double bl, double br) {
        frontLeftMotor.set(fl);
        frontRightMotor.set(fr);
        rearLeftMotor.set(bl);
        rearRightMotor.set(br);

        rearRightMotor.setInverted(true);
    }
    
}
