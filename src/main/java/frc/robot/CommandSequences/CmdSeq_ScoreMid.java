package frc.robot.CommandSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.CmdHybridManipTimed_Outtake;
import frc.robot.Commands.CmdHybridManip_ElevatorGoToPosition;
import frc.robot.Commands.CmdHybridManip_PivotGoToPosition;
import frc.robot.Subsystems.Subsys_Elevator;
import frc.robot.Subsystems.Subsys_Intake;
import frc.robot.Subsystems.Subsys_Pivot;

/**
 * A command sequence that at the same time raises elvator
 * to middle level and opens pivot, shoots out the cube, and
 * then at the same time lowers elevator and retracts pivot.
 */
public class CmdSeq_ScoreMid extends SequentialCommandGroup {
    /**
     * Creates a new CmdSeq_ScoreMid.
     */
    public CmdSeq_ScoreMid(Subsys_Elevator m_elevator, Subsys_Pivot m_pivot, Subsys_Intake m_claw) {
        addCommands(
            new ParallelCommandGroup(
                new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ScoringConstants.kElevatorMidLevel),
                new CmdHybridManip_PivotGoToPosition(m_pivot, 2.3)
            ),
            new CmdHybridManipTimed_Outtake(m_claw, 0.6),
            new ParallelCommandGroup(
                new CmdHybridManip_PivotGoToPosition(m_pivot, 0.2),
                new CmdHybridManip_ElevatorGoToPosition(m_elevator, 0)
            )
        );
    }
}
