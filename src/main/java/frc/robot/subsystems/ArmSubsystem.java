package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.Motor;

public class ArmSubsystem extends SubsystemBase {

    public final static class ArmSubsystemConstants {
        private static double kGearRatio = 0; // determine
        private static double kDistancePerRevolution = 0; // determine
        private static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI *
                        Units.inchesToMeters(kDistancePerRevolution)) * 10));
    }

    Motor armMotor;

    public ArmSubsystem() {
        armMotor = new Motor(RobotMap.ARM_PWM_PIN);
        armMotor.restoreMotorToFactoryDefaults();
        armMotor.resetEncoder();
        armMotor.setSpeedMultiplier(RobotMap.ARM_SPEED_MULTIPLIER);
        armMotor.setEncoderPositionConversionFactor(ArmSubsystemConstants.kLinearDistanceConversionFactor);
        armMotor.setEncoderVelocityConversionFactor(ArmSubsystemConstants.kLinearDistanceConversionFactor / 60);
    }

    public final void openArm() {
        armMotor.setPower(1);
    }

    public final void closeArm() {
        armMotor.setPower(-1);
    }

    public final void stopArm() {
        armMotor.setPower(0);
    }

    public final double getPosition() {
        return armMotor.getEncoderPosition();
    }

    public final boolean isAtPostion(double position) {
        return getPosition() == position;
    }

    public final void resetPosition() {
        armMotor.resetEncoder();
    }
}
