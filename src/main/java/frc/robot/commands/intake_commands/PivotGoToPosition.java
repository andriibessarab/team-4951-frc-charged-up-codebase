package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;

public class PivotGoToPosition extends CommandBase {
    private final PivotSubsystem m_pivot;
    private final double m_position;


    public PivotGoToPosition(PivotSubsystem pivot, double position) {
        m_pivot = pivot;
        m_position = position;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        m_pivot.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        double current = m_pivot.getPosition();
        if (Math.abs(current-m_position) < 0.1) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        m_pivot.setSpeed(0.0);
    }
}
