package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;

public class ClawIntake extends CommandBase{
    private ClawMotorsSubsystem motor;

    public ClawIntake(ClawMotorsSubsystem m){
        motor = m;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        motor.spinIn();
    }

    @Override
    public void end(boolean bool){
        motor.stop();
    }
}
