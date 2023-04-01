package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;

public class ClawOutake extends CommandBase{
    private ClawMotorsSubsystem motor;
    Timer time = new Timer();

    public ClawOutake(ClawMotorsSubsystem m){
        motor = m;
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute(){
        motor.spinOut();
    }

    @Override
    public boolean isFinished() {
        if (time.get()>0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        motor.stop();
    }
}
