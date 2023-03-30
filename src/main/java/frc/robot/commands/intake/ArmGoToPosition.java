package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoToPosition extends CommandBase {
    private final ArmSubsystem m_arm;
    private final double m_position;

    public ArmGoToPosition(ArmSubsystem arm, double position) {
        m_arm = arm;
        m_position = position;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        double current = m_arm.getPosition();
        if (Math.abs(current-m_position) < 0.2) {
            return true;
        }
        return false;
    }
}