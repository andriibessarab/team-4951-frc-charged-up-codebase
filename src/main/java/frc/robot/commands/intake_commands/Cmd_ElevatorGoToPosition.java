package frc.robot.commands.intake_commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;

public class Cmd_ElevatorGoToPosition extends CommandBase {
    private final ElevatorSubsystem m_elevator;
    private final double m_refPosition;

    public Cmd_ElevatorGoToPosition(ElevatorSubsystem elevator, double position) {
        m_elevator = elevator;
        m_refPosition = MathUtil.clamp(position, Constants.ElevatorSubsystemConstants.kMinHeight, Constants.ElevatorSubsystemConstants.kMaxHeight);
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        if (m_refPosition < m_elevator.getPosition()) {
            m_elevator.setSpeed(-0.5);
        } else {
            m_elevator.setSpeed(0.5);
        }
    }

    @Override
    public boolean isFinished() {
        double current = m_elevator.getPosition();
        if (Math.abs(current-m_refPosition) < 0.2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean b){
        m_elevator.setSpeed(0);
    }
}