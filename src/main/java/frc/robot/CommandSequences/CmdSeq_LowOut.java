package frc.robot.CommandSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.CmdHybridManip_ElevatorGoToPosition;
import frc.robot.Commands.CmdHybridManip_PivotGoToPosition;
import frc.robot.Subsystems.Subsys_Elevator;
import frc.robot.Subsystems.Subsys_Pivot;

/**
 * A parallel command that at the same time lowers elevator
 * to bottom level and opens pivot.
 */
public class CmdSeq_LowOut extends ParallelCommandGroup {
    /**
     * Creates a new CmdSeq_LowOut.
     */
    public CmdSeq_LowOut(Subsys_Elevator m_elevator, Subsys_Pivot m_pivot) {
        addCommands(
            new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ScoringConstants.kElevatorBottomLevel),
            new CmdHybridManip_PivotGoToPosition(m_pivot, Constants.ScoringConstants.kPivotOut)
        );
    }
}
