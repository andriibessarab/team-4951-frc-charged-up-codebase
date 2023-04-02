package frc.robot.commands.drivetrain_commands;

import frc.robot.commands.Cmd_DrivetoCharge;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class AutoSQ_NewBalance extends SequentialCommandGroup {
    public AutoSQ_NewBalance(int Where, DriveSubsystem driveTrain) {
        // driveTrain.ResetHeading();
        addCommands(
        //new Cmd_DriveSetPosition(2.3, 3, 180, driveTrain),
        // place the cone
        // new CmdSQ_PlaceConeAuto(Where, driveTrain, Arm, intake),
        // back up 12ft to get out of the community
        new Cmd_DrivetoCharge(driveTrain),
        // drive foward onto the charge station 
        //new Cmd_DriveRotate(1, driveTrain),
        //new CmdSQ_PickupConeFromFloor(driveTrain, Arm, intake),
        //new Cmd_DriveRotate(1, driveTrain),
        //new Cmd_DriveFwd(6.0, driveTrain),
        //Balance
        new Cmd_DriveBalance(driveTrain)
        // new Cmd_DriveRotateToAngle(270, driveTrain)
        
       );
    }
}

