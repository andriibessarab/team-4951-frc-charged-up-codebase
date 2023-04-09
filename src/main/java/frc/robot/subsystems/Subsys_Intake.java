package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClawSubsystemConstants.*;

public class Subsys_Intake extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(kLMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_motor1 = new CANSparkMax(kRMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Subsys_Intake(){
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