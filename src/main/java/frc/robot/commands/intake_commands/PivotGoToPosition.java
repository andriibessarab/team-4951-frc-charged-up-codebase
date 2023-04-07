package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;

public class PivotGoToPosition extends CommandBase {
    private final PivotSubsystem m_pivot;
    private final double m_position;
    Timer time;


    public PivotGoToPosition(PivotSubsystem pivot, double position) {
        m_pivot = pivot;
        m_position = position;
        addRequirements(m_pivot);
        time = new Timer();
    }

    @Override
    public void initialize() {
        m_pivot.setPosition(m_position);
        time.reset();
        time.start();
        
        // if (m_pivot.getPosition() > m_position) {
        //     m_pivot.setSpeed(-0.4);
        // } else {
        //     m_pivot.setSpeed(0.4);
        // }
    }

    @Override
    public boolean isFinished() {
        double current = m_pivot.getPosition();
        if (Math.abs(current-m_position) < 0.2||time.get()>2.0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        m_pivot.stop();
    }
}
