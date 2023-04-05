package frc.robot.commands.drivetrain_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;

public class Cmd_LeaveCommunityZone extends CommandBase {
    private final DriveSubsystem m_drive;
    private final boolean m_positionedForward;
    Timer m_timer = new Timer();

    public Cmd_LeaveCommunityZone(DriveSubsystem drive, boolean positionedForward) {
        m_drive = drive;
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
            (m_positionedForward ? 0.3 : -0.3),
            0
        );
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > 2.2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        m_drive.driveMecanum(0, 0, 0);
    }
}