package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class PneumaticsSubsystem extends SubsystemBase{

    private DoubleSolenoid intake;
    private Value state;

    public PneumaticsSubsystem(){
        intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.INTAKE_FORWRD_CHANNEL, RobotMap.INTAKE_REVERSE_CHANNEL);
        intake.set(Value.kReverse);
        state = Value.kReverse;
    }

    public void move(){
        if(state==Value.kReverse){
            state = Value.kForward;
        } else{
            state=Value.kReverse;
        }
        intake.set(state);
    }

    public boolean getStatus(){
        //false if retracted, true if extended
        return state==Value.kForward;
    }
}
