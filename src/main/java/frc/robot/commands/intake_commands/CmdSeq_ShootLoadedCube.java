package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake_subsystems.ClawMotorsSubsystem;
import frc.robot.subsystems.intake_subsystems.ElevatorSubsystem;
import frc.robot.subsystems.intake_subsystems.PivotSubsystem;

public class CmdSeq_ShootLoadedCube extends SequentialCommandGroup {
    public CmdSeq_ShootLoadedCube(ElevatorSubsystem m_elevator, PivotSubsystem m_pivot, ClawMotorsSubsystem m_claw) {
        super (
            // Raise elevator & open pivot simoltaneously
            new ParallelCommandGroup(
                new Cmd_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kTopLayerHeight),
                new Cmd_PivotGoToPosition(m_pivot, 1.7)
            ),

            // Shoot out the cube
            new Cmd_ClawOuttake(m_claw),

            // Lower elevator & close pivot simoltaneously
            new ParallelCommandGroup(
                new Cmd_PivotGoToPosition(m_pivot, 0),
                new Cmd_ElevatorGoToPosition(m_elevator, 0.0)
            )
        );
    }
}
