package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain_commands.Cmd_LeaveCommunityZone;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;

public class CmdSeqAuton_Drive extends SequentialCommandGroup {
    public CmdSeqAuton_Drive(DriveSubsystem m_drive) {
        super (
            // Leave community zone
            new Cmd_LeaveCommunityZone(m_drive, true)
        );
    }
}
