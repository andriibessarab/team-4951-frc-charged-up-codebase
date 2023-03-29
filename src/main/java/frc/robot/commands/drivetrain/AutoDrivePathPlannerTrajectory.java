package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrivePathPlannerTrajectory extends SequentialCommandGroup {
    public AutoDrivePathPlannerTrajectory(DriveSubsystem robotDrive, String pathName, boolean isFirstPath, double maxVelocity, double maxAccel) {

        boolean fieldRelative = true;

        var trajectory = PathPlanner.loadPath(pathName, maxVelocity, maxAccel);

        var mecanumControllerPath = new PPMecanumControllerCommand(
                trajectory,
                robotDrive::getPose, // Pose supplier
                Constants.DriveConstants.kDriveKinematics, // MecanumDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                3.0, // Max wheel velocity meters per second
                robotDrive::setCurrentWheelSpeeds, // MecanumDriveWheelSpeeds consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                robotDrive // Requires this drive subsystem
        );

        addCommands(
                new InstantCommand(() -> {
                    if (isFirstPath) {
                        robotDrive.resetOdometry(trajectory.getInitialHolonomicPose());
                    }
                }),
                mecanumControllerPath,
                new InstantCommand(() -> {
                    robotDrive.drive(0.0, 0.0, 0.0, fieldRelative);
                }));
    }
}