package frc.robot.commands.drivetrain_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;

public class Cmd_DriveFwdForSetTIme extends CommandBase {
    private final DriveSubsystem m_drive;
    private final double m_driveTime;
    private final boolean m_positionedForward;
    Timer m_timer = new Timer();

    public Cmd_DriveFwdForSetTIme(DriveSubsystem drive, double driveTime, boolean positionedForward) {
        m_drive = drive;
        m_driveTime = driveTime;
        m_positionedForward = positionedForward;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_drive.driveMecanum(
            0,
            (m_positionedForward ? 0.4 : -0.4),
            0
        );
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > m_driveTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        m_drive.driveMecanum(0, 0, 0);
    }
}
