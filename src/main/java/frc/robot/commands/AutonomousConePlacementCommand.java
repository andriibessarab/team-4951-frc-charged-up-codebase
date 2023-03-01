package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

// Put current/cone/cube in place

public class AutonomousConePlacementCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public AutonomousConePlacementCommand(DrivetrainSubsystem drivetrainSubsystem) {
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
