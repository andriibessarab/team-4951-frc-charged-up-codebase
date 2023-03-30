package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;

public class ElevatorGotoPosition extends CommandBase {
    private final ElevatorSubsystem m_elevator;
    private final double m_position;

    public ElevatorGotoPosition(ElevatorSubsystem elevator, double position) {
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
}