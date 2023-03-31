package frc.robot.subsystems.intake_subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClawPneumaticsSubsystem {
    private final DoubleSolenoid reach = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ClawSubsystemConstants.forwardChannel, Constants.ClawSubsystemConstants.reverseChannel);
    private Value state = Value.kReverse;
    public ClawPneumaticsSubsystem(){
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
