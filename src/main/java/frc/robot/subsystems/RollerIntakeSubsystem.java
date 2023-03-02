package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.Motor;

public class RollerIntakeSubsystem extends SubsystemBase {

    public final static class ArmSubsystemConstants {
        private static double kGearRatio = 0; // determine
        private static double kDistancePerRevolution = 0; // determine
        private static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI *
                        Units.inchesToMeters(kDistancePerRevolution)) * 10));
    }

    Motor intakeMotor;

    public RollerIntakeSubsystem() {
        intakeMotor = new Motor(RobotMap.INTAKE_PWM_PIN);
        intakeMotor.restoreMotorToFactoryDefaults();
        intakeMotor.resetEncoder();
        intakeMotor.setSpeedMultiplier(RobotMap.INTAKE_SPEED_MULTIPLIER);
        intakeMotor.setEncoderPositionConversionFactor(ArmSubsystemConstants.kLinearDistanceConversionFactor);
        intakeMotor.setEncoderVelocityConversionFactor(ArmSubsystemConstants.kLinearDistanceConversionFactor / 60);
    }

    public final void intakeOut() {
        intakeMotor.setPower(1);
    }

    public final void intakeIn() {
        intakeMotor.setPower(-1);
    }

    public final void intakeStop() {
        intakeMotor.setPower(0);
    }

    public final double getPosition() {
        return intakeMotor.getEncoderPosition();
    }

    public final boolean isAtPostion(double position) {
        return getPosition() == position;
    }

    public final void resetPosition() {
        intakeMotor.resetEncoder();
    }
}
