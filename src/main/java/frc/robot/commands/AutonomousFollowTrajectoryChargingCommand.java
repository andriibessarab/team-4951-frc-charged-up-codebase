package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

// Follow trajectory to charging station

public class AutonomousFollowTrajectoryChargingCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;

  public AutonomousFollowTrajectoryChargingCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
