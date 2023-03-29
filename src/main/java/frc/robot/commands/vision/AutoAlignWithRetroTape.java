package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignWithRetroTape extends CommandBase {
    private LimelightSubsystem m_limelight;
    private DriveSubsystem m_drive;

    public AutoAlignWithRetroTape(LimelightSubsystem limelight, DriveSubsystem drive) {
        m_limelight = limelight;
        m_drive = drive;
        addRequirements(m_limelight);
        addRequirements(m_drive);        // Takes control of drive system
    }

    @Override
    public void initialize() {
        // Switch to post alignment pipeline from what ever it might have been
        m_limelight.activatePipeline(Constants.LimelightSubsystem.kRetroTapePostAlignmentPipelineID);
    }

    @Override
    public void execute() {
        // https://docs.limelightvision.io/en/latest/cs_seeking.html
        if (!m_limelight.isTargetFound()) {
            // We don't see the target, spin clockwise for the target by spinning in place
            // at a safe speed.
            m_drive.drive(0.0, 0.0, Constants.Vision.kSpinClockwiseSearchRate, true);
        } else {
            // We do see the target, execute aiming code
            var horizontalOffset = m_limelight.getTargetX();  // LL2: -29.8 to 29.8 degrees

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
