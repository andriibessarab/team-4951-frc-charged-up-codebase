package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;

public class Claw extends CommandBase{
    private ClawMotorsSubsystem motor;

    public Claw(ClawMotorsSubsystem m){
        motor = m;
    }

    @Override
    public void initialize() {
        motor.spin();
    }

    @Override
    public void end(boolean bool){
        motor.stop();
    }
}
