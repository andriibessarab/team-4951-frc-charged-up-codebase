package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Subsys_Drive;

public class CmdAutonDrive_LeaveCommZone extends CommandBase {
    private final Subsys_Drive m_drive;
    Timer m_timer = new Timer();

    public CmdAutonDrive_LeaveCommZone(Subsys_Drive drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {

        if(m_timer.get()<2){
            m_drive.driveMecanum(0, 0, 0);
        } else{
            m_drive.driveMecanum(0, -0.3, 0);
        }
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > 4.2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        m_drive.driveMecanum(0, 0, 0);
    }
}