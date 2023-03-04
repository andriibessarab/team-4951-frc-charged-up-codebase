package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


/**
 * Responsible for raising.lowering the arm and intake system
 */
public class ElevatorSubsystem extends SubsystemBase {
/* 
    public final static class ElevatorSubsystemConstants {
        private static double kGearRatio = 0; // determine
        private static double kDistancePerRevolution = 0; // determine
        private static final double kLinearDistanceConversionFactor = (Units
                .inchesToMeters(1 / (kGearRatio * 2 * Math.PI *
                        Units.inchesToMeters(kDistancePerRevolution)) * 10));
    }

    Motor elevatorMotor;

    public ElevatorSubsystem() {
        elevatorMotor = new Motor(RobotMap.ELEVATOR_PWM_PIN);
        elevatorMotor.restoreMotorToFactoryDefaults();
        elevatorMotor.resetEncoder();
        elevatorMotor.setSpeedMultiplier(RobotMap.ELEVATOR_SPEED_MULTIPLIER);
        elevatorMotor.setEncoderPositionConversionFactor(ElevatorSubsystemConstants.kLinearDistanceConversionFactor);
        elevatorMotor
                .setEncoderVelocityConversionFactor(ElevatorSubsystemConstants.kLinearDistanceConversionFactor / 60);
    }

    public final void raiseElevator() {
        elevatorMotor.setPower(1);
    }

    public final void lowerElevator() {
        elevatorMotor.setPower(-1);
    }

    public final void stopElevator() {
        elevatorMotor.setPower(0);
    }

    public final double getPosition() {
        return elevatorMotor.getEncoderPosition();
    }

    public final boolean isAtPostion(double position) {
        return getPosition() == position;
    }

    public final void resetPosition() {
        elevatorMotor.resetEncoder();
    }*/
}
