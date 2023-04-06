package frc.robot.commands.autonomous_commands;

import frc.robot.Constants;
import frc.robot.commands.drivetrain_commands.Cmd_DriveBalance;
import frc.robot.commands.drivetrain_commands.Cmd_DriveFwdForSetTIme;
import frc.robot.commands.drivetrain_commands.Cmd_DrivetoCharge;
import frc.robot.commands.intake_commands.CmdSeq_ShootLoadedCube;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;
import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class CmdSeqAuton_ShootCubeBalance extends SequentialCommandGroup {
    public CmdSeqAuton_ShootCubeBalance(DriveSubsystem m_drive, ElevatorSubsystem m_elevator, PivotSubsystem m_pivot, ClawMotorsSubsystem m_claw) {
        addCommands(
            //new Cmd_DriveSetPosition(2.3, 3, 180, driveTrain),

            // Shoot out pre-loaded cube
            new CmdSeq_ShootLoadedCube(m_elevator, m_pivot, m_claw),

            // Drive to charging station and get on it before running next method
            new Cmd_DriveFwdForSetTIme(m_drive, 4, false),

            // 
            new Cmd_DriveBalance(m_drive)

            // new Cmd_DriveRotateToAngle(270, driveTrain)
       );
    }
}

