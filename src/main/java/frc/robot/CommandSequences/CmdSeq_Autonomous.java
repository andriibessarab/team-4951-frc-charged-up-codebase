package frc.robot.CommandSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.CmdAutonDrive_LeaveCommZone;
import frc.robot.Commands.CmdHybridManipTimed_Outtake;
import frc.robot.Commands.CmdHybridManip_ElevatorGoToPosition;
import frc.robot.Commands.CmdHybridManip_PivotGoToPosition;
import frc.robot.Subsystems.Subsys_Drive;
import frc.robot.Subsystems.Subsys_Elevator;
import frc.robot.Subsystems.Subsys_Intake;
import frc.robot.Subsystems.Subsys_Pivot;

/**
 * A command sequence for autonoous
 */
public class CmdSeq_Autonomous extends SequentialCommandGroup {
    /**
     * Creates a new CmdSeq_Autonomous.
     */
    public CmdSeq_Autonomous(Subsys_Drive m_drive, Subsys_Elevator m_elevator, Subsys_Pivot m_pivot, Subsys_Intake m_claw) {
        addCommands(
            new ParallelCommandGroup(
                new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ScoringConstants.kElevatorTopLevel),
                new CmdHybridManip_PivotGoToPosition(m_pivot, Constants.ScoringConstants.kPivotOut)
            ),
            new CmdHybridManipTimed_Outtake(m_claw, Constants.ScoringConstants.kOuttakeSpeedTopLevel),
            new ParallelCommandGroup(
                new CmdAutonDrive_LeaveCommZone(m_drive),
                new CmdHybridManip_PivotGoToPosition(m_pivot, Constants.ScoringConstants.kPivotIn),
                new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ScoringConstants.kElevatorBottomLevel)
            )
        );
    }
}
