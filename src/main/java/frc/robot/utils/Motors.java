package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.RobotMap;

public class Motors {

    private final static Victor rearLeftMotor = new Victor(RobotMap.REAR_LEFT_MOTOR);
    private final static Victor rearRightMotor = new Victor(RobotMap.REAR_RIGHT_MOTOR);
    private final static Victor frontLeftMotor = new Victor(RobotMap.FRONT_LEFT_MOTOR);
    private final static Victor frontRightMotor = new Victor(RobotMap.FRONT_RIGHT_MOTOR);

    // Variables
    private static double speedMultiplier = 1;

    // Set speed multiplier for motors
    public final static void setSpeed(double speed) {
        // Check that 0 < speed <= 1
        if (speed > 1)
            speedMultiplier = 1;
        else if (speed < 0)
            speedMultiplier = 0;
        else
            speedMultiplier = speed;
    }

    public final static void initialize() {
        rearRightMotor.setInverted(true);
    }

    public final static void setPower(double fl, double fr, double rl, double rr) {
        frontLeftMotor.set(fl * speedMultiplier);
        frontRightMotor.set(fr * speedMultiplier);
        rearLeftMotor.set(rl * speedMultiplier);
        rearRightMotor.set(rr * speedMultiplier);
    }
}
