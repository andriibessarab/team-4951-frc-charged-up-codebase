package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticCommand extends CommandBase{
    private PneumaticsSubsystem pSubsystem;
    private boolean moved = false;
    public PneumaticCommand(PneumaticsSubsystem pSubsystem) {
        this.pSubsystem = pSubsystem;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        pSubsystem.move();
        moved = true;
    }

    @Override
    public boolean isFinished() {
        return moved;
    }

    @Override
    public void end(boolean interrupted) {}
}
