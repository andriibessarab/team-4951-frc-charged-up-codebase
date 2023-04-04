package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawMotorsSubsystem extends SubsystemBase{
    private final CANSparkMax m_left_motor = new CANSparkMax(Constants.ClawSubsystemConstants.kLMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_right_motor = new CANSparkMax(Constants.ClawSubsystemConstants.kRMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ClawMotorsSubsystem(){
        m_left_motor.restoreFactoryDefaults();
        m_right_motor.restoreFactoryDefaults();

        m_left_motor.setInverted(Constants.ClawSubsystemConstants.kLeftMotorInverted);
        m_right_motor.setInverted(Constants.ClawSubsystemConstants.kRightMotorInverted);

        m_left_motor.setSmartCurrentLimit(20);
        m_right_motor.setSmartCurrentLimit(20);

        m_left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void spinIn(){
        m_left_motor.set(Constants.ClawSubsystemConstants.kMotorsSpinInSpeed);
        m_right_motor.set(Constants.ClawSubsystemConstants.kMotorsSpinInSpeed);
    }

    public void spinOut(){
        m_left_motor.set(Constants.ClawSubsystemConstants.kMotorsSpinOutSpeed);
        m_right_motor.set(Constants.ClawSubsystemConstants.kMotorsSpinOutSpeed);
    }

    public void stop(){
        m_left_motor.stopMotor();
        m_right_motor.stopMotor();
    }
}
