package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem.DrivetrainSubsystemBase;

/** 
 * Balance on charging station
 */ 
public class BalanceOnStationCommand extends CommandBase {
  private final DrivetrainSubsystemBase drivetrainSubsystem;
  private boolean balanced = false;

  public BalanceOnStationCommand(DrivetrainSubsystemBase drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    balanced = false;
  }

  @Override
  public void execute() {
    balanced = drivetrainSubsystem.balanceOnStation();
  }

  @Override
  public boolean isFinished() {
    return balanced;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
