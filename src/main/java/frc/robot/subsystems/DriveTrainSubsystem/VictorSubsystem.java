package frc.robot.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj.motorcontrol.Victor;

public class VictorSubsystem extends DriveSubsystem{
    
    // The motor objects used for controlling the robot's drivetrain
    public Victor rearLeftMotor;
    public Victor rearRightMotor;
    public Victor frontLeftMotor;
    public Victor frontRightMotor;

    public void setMotorSpeeds(double fl, double fr, double bl, double br) {
        frontLeftMotor.set(fl);
        frontRightMotor.set(fr);
        rearLeftMotor.set(bl);
        rearRightMotor.set(br);

        rearRightMotor.setInverted(true);
    }
    
}
