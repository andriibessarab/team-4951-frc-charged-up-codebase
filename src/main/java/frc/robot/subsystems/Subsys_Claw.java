package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClawSubsystemConstants.*;

public class Subsys_Claw extends SubsystemBase{
    private final DoubleSolenoid reach = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, kForwardChannel, kReverseChannel);
    private Value state = Value.kReverse;

    public Subsys_Claw(){
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