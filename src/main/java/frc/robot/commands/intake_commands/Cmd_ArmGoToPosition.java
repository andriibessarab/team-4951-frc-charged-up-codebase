package frc.robot.commands.intake_commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake_subsystems.ArmSubsystem;

public class Cmd_ArmGoToPosition extends CommandBase {
    private final ArmSubsystem m_arm;
    private final double m_refPosition;

    public Cmd_ArmGoToPosition(ArmSubsystem arm, double position) {
        m_arm = arm;
        m_refPosition = MathUtil.clamp(position, Constants.ElevatorSubsystemConstants.kMinHeight, Constants.ElevatorSubsystemConstants.kMaxHeight);
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        if (m_refPosition < m_arm.getPosition()) {
            m_arm.setSpeed(-0.5);
        } else {
            m_arm.setSpeed(0.5);
        }
    }

    @Override
    public boolean isFinished() {
        double current = m_arm.getPosition();
        if (Math.abs(current-m_refPosition) < 0.2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean b){
        m_arm.setSpeed(0);
    }
}