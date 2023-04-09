package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Subsys_Intake;

public class CmdHybridManipTimed_Outtake extends CommandBase{
    private Subsys_Intake motor;
    Timer time = new Timer();
    double m_speed;

    public CmdHybridManipTimed_Outtake(Subsys_Intake m, double speed){
        motor = m;
        m_speed = speed;
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute(){
        motor.spinOut(m_speed);
    }

    @Override
    public boolean isFinished() {
        if (time.get()>0.75) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean bool){
        motor.stop();
    }
}