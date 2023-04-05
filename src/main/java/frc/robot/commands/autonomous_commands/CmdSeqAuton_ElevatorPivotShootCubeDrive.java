package frc.robot.commands.autonomous_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain_commands.Cmd_LeaveCommunityZone;
import frc.robot.commands.intake_commands.CmdSeq_ShootLoadedCube;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;
import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;

public class CmdSeqAuton_ElevatorPivotShootCubeDrive extends SequentialCommandGroup {
    public CmdSeqAuton_ElevatorPivotShootCubeDrive(DriveSubsystem m_drive, ElevatorSubsystem m_elevator, PivotSubsystem m_pivot, ClawMotorsSubsystem m_claw) {
        super (
            // Shoot loaded cone sequence
            new CmdSeq_ShootLoadedCube(m_elevator, m_pivot, m_claw),

            // Leave community zone
            new Cmd_LeaveCommunityZone(m_drive, false)
        );
    }
}
