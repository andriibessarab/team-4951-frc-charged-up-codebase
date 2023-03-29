package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Claw_Pneumatic {
    private final DoubleSolenoid reach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Claw.forwardChannel, Constants.Claw.reverseChannel);
    private Value state = Value.kReverse;
    public Claw_Pneumatic(){
        state = Value.kReverse;
        reach.set(state);
    }
    public void use(){
        if(state==Value.kReverse){
            state = Value.kForward;
        } else{
            state = Value.kReverse;
        }
        reach.set(state);
    }
}
