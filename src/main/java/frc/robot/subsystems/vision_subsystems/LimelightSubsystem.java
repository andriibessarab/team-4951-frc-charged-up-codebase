package frc.robot.subsystems.vision_subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.helpers.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
    private String m_limelightName;

    public LimelightSubsystem(String limelightName) {
        m_limelightName = limelightName;
    }

    public String getLimelightName() {
        return m_limelightName;
    }

    public void activatePipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(getLimelightName(), pipeline);
    }

    public boolean isTargetFound() {
        return LimelightHelpers.getTV(getLimelightName());
    }

    public double getTargetX() {
        return LimelightHelpers.getTX(getLimelightName());
    }

    public double getTargetY() {
        return LimelightHelpers.getTY(getLimelightName());
    }

    public Pose2d getBotPose2d() {
        // Could check if something in view - but unless using getLatestResults()
        // to atomically have all the data and consume 2.5ms, by the time I read
        // the pose it was sometimes missing.
        double[] botPose = LimelightHelpers.getBotPose(getLimelightName());

        // If Limelight having difficulty, it may not be able to present the Pose (ie. not connected)
        if (botPose.length != 6) {
            return null;
        }

        if (botPose[0] == 0.0 && botPose[1] == 0.0 && botPose[3] == 0.0 &&
                botPose[4] == 0.0 && botPose[5] == 0.0 && botPose[6] == 0.0) {
            return null;
        }
        Translation2d tran2d = new Translation2d(botPose[0], botPose[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(botPose[5]));
        return new Pose2d(tran2d, r2d);
    }

    public double getLatency_Pipeline() {
        return LimelightHelpers.getLatency_Pipeline(getLimelightName());
    }

    public LimelightHelpers.LimelightResults getLatestResults() {
        // This can take upwards of 2.5ms
        return LimelightHelpers.getLatestResults(getLimelightName());
    }
}
