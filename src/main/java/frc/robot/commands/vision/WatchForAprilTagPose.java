package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class WatchForAprilTagPose extends CommandBase {
    private LimelightSubsystem m_limelight;
    private DriveSubsystem m_drive;

    public WatchForAprilTagPose(LimelightSubsystem limelight, DriveSubsystem drive) {
        m_limelight = limelight;
        m_drive = drive;  // NOTE: Passively notify of AprilTag finds (do NOT invoke drive)
        addRequirements(m_limelight);
    }

    @Override
    public void initialize() {
        // Switch to AprilTags pipeline from what ever it might have been
        m_limelight.activatePipeline(Constants.LimelightSubsystem.kAprilTagsWatchPosePipelineID);
    }

    @Override
    public void execute() {
        Pose2d pose = m_limelight.getBotPose2d();   // Returns field pose (relative to center) or null
        if ( pose != null ) {
//            m_drive.resetOdometry(pose);
        }
    }
}
