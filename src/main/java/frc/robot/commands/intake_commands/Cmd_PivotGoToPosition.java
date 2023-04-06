package frc.robot.commands.intake_commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;

public class Cmd_PivotGoToPosition extends CommandBase {
    private final PivotSubsystem m_pivot;
    private final double m_refPosition;

    public Cmd_PivotGoToPosition(PivotSubsystem pivot, double position) {
        m_pivot = pivot;
        m_refPosition = MathUtil.clamp(position, Constants.ElevatorSubsystemConstants.kMinHeight, Constants.ElevatorSubsystemConstants.kMaxHeight);
        addRequirements(m_pivot);
    }

    @Override
    public void execute() {
        // if (m_refPosition < m_pivot.getPosition()) {
        //     m_pivot.setSpeed(-0.5);
        // } else {
        //     m_pivot.setSpeed(0.5);
        // }

        m_pivot.setPosition(m_refPosition);
    }

    @Override
    public boolean isFinished() {
        double current = m_pivot.getPosition();
        if (Math.abs(current-m_refPosition) < 0.2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean b){
        m_pivot.setSpeed(0);
    }
}


