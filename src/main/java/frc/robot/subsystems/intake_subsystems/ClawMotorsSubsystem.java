package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;

import frc.robot.Constants;

public class ClawMotorsSubsystem {
    private final CANSparkMax m_motor = new CANSparkMax(Constants.Claw.kLMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motor1 = new CANSparkMax(Constants.Claw.kRMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ClawMotorsSubsystem(){
        m_motor.setInverted(true);

        m_motor.setSmartCurrentLimit(10);
        m_motor1.setSmartCurrentLimit(10);

        m_motor.restoreFactoryDefaults();
        m_motor1.restoreFactoryDefaults();

        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    public void spinIn(){
        m_motor.set(0.5);
        m_motor1.set(0.5);
    }
    public void spinOut(){
        m_motor.set(-0.5);
        m_motor1.set(-0.5);
    }
    public void stop(){
        m_motor.stopMotor();
        m_motor1.stopMotor();
    }
}
