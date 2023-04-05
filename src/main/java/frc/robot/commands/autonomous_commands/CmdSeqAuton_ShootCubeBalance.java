// package frc.robot.commands.autonomous_commands;

// import frc.robot.Constants;
// import frc.robot.commands.drivetrain_commands.Cmd_DriveBalance;
// import frc.robot.commands.drivetrain_commands.Cmd_DrivetoCharge;
// import frc.robot.commands.intake_commands.claw_commands.ClawOutake;
// import frc.robot.commands.intake_commands.elevator_commands.ElevatorGotoDown;
// import frc.robot.commands.intake_commands.elevator_commands.ElevatorGotoPosition;
// import frc.robot.commands.intake_commands.pivot_commands.PivotClose;
// import frc.robot.commands.intake_commands.pivot_commands.PivotGoToPosition;
// import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;
// import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;
// import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.intake_subsystems.PivotSubsystem;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


// public class UltimateAutonomousCommandGroup extends SequentialCommandGroup {
//     public UltimateAutonomousCommandGroup(int Where, DriveSubsystem m_drive, ElevatorSubsystem m_elevator, PivotSubsystem m_pivot, ClawMotorsSubsystem m_claw) {
//         // driveTrain.ResetHeading();
//         addCommands(
//             //new Cmd_DriveSetPosition(2.3, 3, 180, driveTrain),

//             // Shoot out pre-loaded cube
//             new ElevatorGotoPosition(m_elevator, Constants.ElevatorSubsystemConstants.kTopLayerHeight),
//             new PivotGoToPosition(m_pivot, 1.7),
//             new ClawOutake(m_claw),
//             new PivotClose(m_pivot),
//             new ElevatorGotoDown(m_elevator, 0.0),

//             // back up 12ft to get out of the community
//             new LeaveCommunityZoneCommand(m_drive),

//             new Cmd_DrivetoCharge(driveTrain),
//             // drive foward onto the charge station 
//             //new Cmd_DriveRotate(1, driveTrain),
//             //new CmdSQ_PickupConeFromFloor(driveTrain, Arm, intake),
//             //new Cmd_DriveRotate(1, driveTrain),
//             //new Cmd_DriveFwd(6.0, driveTrain),
//             //Balance
//             new Cmd_DriveBalance(driveTrain)
//             // new Cmd_DriveRotateToAngle(270, driveTrain)
//        );
//     }
// }

