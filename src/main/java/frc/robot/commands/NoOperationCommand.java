package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

// Does nothing - just finishes immediately
public class NoOperationCommand extends CommandBase {

    public NoOperationCommand() {
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
