package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Subsys_Intake;

public class CmdHybridManip_Intake extends CommandBase{
    private Subsys_Intake motor;

    public CmdHybridManip_Intake(Subsys_Intake m){
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