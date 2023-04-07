package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawMotorsSubsystem extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(Constants.ClawSubsystemConstants.kLMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motor1 = new CANSparkMax(Constants.ClawSubsystemConstants.kRMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ClawMotorsSubsystem(){
        m_motor.restoreFactoryDefaults();
        m_motor1.restoreFactoryDefaults();

        m_motor.setInverted(true);

        m_motor.setSmartCurrentLimit(20);
        m_motor1.setSmartCurrentLimit(20);

        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    public void spinIn(){
        m_motor.set(0.25);
        m_motor1.set(0.25);
    }
    public void spinOut(double speed){
        m_motor.set(-speed);
        m_motor1.set(-speed);
    }
    public void stop(){
        m_motor.stopMotor();
        m_motor1.stopMotor();
    }
}
