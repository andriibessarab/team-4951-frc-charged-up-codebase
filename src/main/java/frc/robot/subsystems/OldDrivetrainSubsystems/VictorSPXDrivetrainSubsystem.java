package frc.robot.Subsystems.OldDrivetrainSubsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.RobotMap;

public class VictorSPXDrivetrainSubsystem extends MecanumDrivetrainSubsystemBase{
    
    // The motor objects used for controlling the robot's drivetrain
    private Victor rearLeftMotor;
    private Victor rearRightMotor;
    private Victor frontLeftMotor;
    private Victor frontRightMotor;

    public VictorSPXDrivetrainSubsystem() {
        super();
        this.rearLeftMotor = new Victor(RobotMap.REAR_LEFT_MOTOR_PWM_PIN);
        this.rearRightMotor = new Victor(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN);
        this.frontLeftMotor = new Victor(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN);
        this.frontRightMotor = new Victor(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN);
    }

    public void setMotorSpeeds(double fl, double fr, double bl, double br) {
        frontLeftMotor.set(fl);
        frontRightMotor.set(fr);
        rearLeftMotor.set(bl);
        rearRightMotor.set(br);

        rearRightMotor.setInverted(true);
    }
    
}
