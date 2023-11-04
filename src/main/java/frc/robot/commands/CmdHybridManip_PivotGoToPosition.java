package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Subsys_Pivot;

public class CmdHybridManip_PivotGoToPosition extends CommandBase {
    private final Subsys_Pivot m_pivot;
    private final double m_position;
    Timer m_timer = new Timer();


    public CmdHybridManip_PivotGoToPosition(Subsys_Pivot pivot, double position) {
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
        return Math.abs(current-m_position) < 0.1 || m_timer.get() > 2;
    }

    @Override
    public void end(boolean bool){
        m_pivot.stop();
    }
}