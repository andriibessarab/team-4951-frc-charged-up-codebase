package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPosition extends CommandBase {
    private final PivotSubsystem m_pivot;
    private final double m_position;


    public PivotPosition(PivotSubsystem pivot, boolean pivotOut) {
        m_pivot = pivot;
        m_position = pivotOut ? frc.robot.Constants.PivotSubsystem.kMaxOut : frc.robot.Constants.PivotSubsystem.kMinOut;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        m_pivot.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        double current = m_pivot.getPosition();
        if (Math.abs(current-m_position) < 0.2) {
            return true;
        }
        return false;
    }
}