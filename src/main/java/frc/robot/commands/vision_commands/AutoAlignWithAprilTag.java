package frc.robot.commands.vision_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain_subsystems.DriveSubsystem;
import frc.robot.subsystems.vision_subsystems.LimelightSubsystem;

public class AutoAlignWithAprilTag extends CommandBase {
    private LimelightSubsystem m_limelight;
    private DriveSubsystem m_drive;

    private long m_lastTargetTimestampNS;
    private long consecutiveMissing = 0;
    private long maxConsecutiveMissing = 0;

    public AutoAlignWithAprilTag(LimelightSubsystem limelight, DriveSubsystem drive) {
        m_limelight = limelight;
        m_drive = drive;
        addRequirements(m_limelight);
        addRequirements(m_drive);        // Takes control of drive system
    }

    @Override
    public void initialize() {
        // Switch to post alignment pipeline from what ever it might have been
        m_limelight.activatePipeline(Constants.LimelightSubsystem.kAprilTagsPostAlignmentPipelineID);
    }

    @Override
    public void execute() {
        long now = System.nanoTime();
        // https://docs.limelightvision.io/en/latest/cs_seeking.html
        if (!m_limelight.isTargetFound()) {
            var heading = m_drive.getHeading();
            //System.out.println("[" + now + "] Target M: "+heading+","+consecutiveMissing);

            consecutiveMissing++;
            // We don't see the target, spin clockwise for the target by spinning in place
            // at a safe speed.
            long deltaMissingTimeNS = System.nanoTime() - m_lastTargetTimestampNS;
            final long updateCycleInNS = 20*1000000;
            if ( deltaMissingTimeNS > updateCycleInNS*15 ) {  // If missed 15 cycles (300ms) make a move
                m_drive.drive(0.0, 0.0, Constants.Vision.kSpinClockwiseSearchRate, true);
            } else {
                m_drive.drive(0.0, 0.0, 0.0, true);
            }
        } else {
            m_lastTargetTimestampNS = System.nanoTime();
            if ( consecutiveMissing > maxConsecutiveMissing ) {
                maxConsecutiveMissing = consecutiveMissing;
            }
            consecutiveMissing = 0;

            // We do see the target, execute aiming code
            var horizontalOffset = m_limelight.getTargetX();  // LL2: -29.8 to 29.8 degrees
            var heading = m_drive.getHeading();
            //System.out.println("[" + now + "] Target F: "+horizontalOffset + ", " + heading);

            if (Math.abs(horizontalOffset) > Constants.Vision.kAlignmentKP) {
                horizontalOffset *= Constants.Vision.kAlignmentKP;
            }
            m_drive.drive(0.0, 0.0, horizontalOffset, true);

            // https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
            // To drive closer we could estimate the distances if we do the math and have
            // the correct heights configured to provide an xspeed.
        }
    }
}
