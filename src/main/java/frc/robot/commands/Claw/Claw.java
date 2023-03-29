package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw_Motors;

public class Claw extends CommandBase{
    private Claw_Motors motor;

    public Claw(Claw_Motors m){
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
