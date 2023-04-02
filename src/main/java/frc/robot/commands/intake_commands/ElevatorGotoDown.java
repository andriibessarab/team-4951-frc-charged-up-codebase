package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;

public class ElevatorGotoDown extends CommandBase {
    private final ElevatorSubsystem m_elevator;
    private final double m_position;
    Timer time = new Timer();
    double m_runTime = 2.0;
    double m_speed = -0.2;

    public ElevatorGotoDown(ElevatorSubsystem elevator, double position) {
        m_elevator = elevator;
        m_position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute(){
        m_elevator.setSpeed1(m_speed);
    }

    @Override
    public boolean isFinished() {
        if (time.get()>m_runTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        m_elevator.stop1();
    }
}
