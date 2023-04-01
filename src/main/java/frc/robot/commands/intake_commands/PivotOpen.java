package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;

public class PivotOpen extends CommandBase{
    private PivotSubsystem motor;
    Timer time = new Timer();

    public PivotOpen(PivotSubsystem m){
        motor = m;
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute(){
        motor.setSpeed(0.2);
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
