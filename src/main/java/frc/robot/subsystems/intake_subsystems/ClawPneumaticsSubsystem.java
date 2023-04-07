package frc.robot.subsystems.intake_subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPneumaticsSubsystem extends SubsystemBase{
    private final DoubleSolenoid reach = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, Constants.ClawSubsystemConstants.forwardChannel, Constants.ClawSubsystemConstants.reverseChannel);
    private Value state = Value.kReverse;
    public ClawPneumaticsSubsystem(){
        state = Value.kForward;
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
