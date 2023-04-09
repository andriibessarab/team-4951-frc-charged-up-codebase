package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Subsys_Elevator;

public class CmdHybridManip_ElevatorGoToPosition extends CommandBase {
    private final Subsys_Elevator m_elevator;
    private final double m_position;

    public CmdHybridManip_ElevatorGoToPosition(Subsys_Elevator elevator, double position) {
        m_elevator = elevator;
        m_position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setPosition(m_position);
    }

    @Override
    public boolean isFinished() {
        double current = m_elevator.getPosition();
        if (Math.abs(current-m_position) < 0.2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean b){
    }
}