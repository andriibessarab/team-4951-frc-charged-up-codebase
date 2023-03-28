package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

public class MecanumDriveExample extends SequentialCommandGroup {
    public MecanumDriveExample(DriveSubsystem robotDrive, boolean fieldRelative) {
        addRequirements(robotDrive);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoDriveConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoDriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        config);

        MecanumControllerCommand mecanumControllerCommand =
                new MecanumControllerCommand(
                        exampleTrajectory,
                        robotDrive::getPose,
                        Constants.DriveConstants.kFeedforward,
                        Constants.DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(Constants.AutoDriveConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoDriveConstants.kPYController, 0, 0),
                        new ProfiledPIDController(
                                Constants.AutoDriveConstants.kPThetaController, 0, 0,
                                Constants.AutoDriveConstants.kThetaControllerConstraints),

                        // Needed for normalizing wheel speeds
                        Constants.AutoDriveConstants.kMaxSpeedMetersPerSecond,

                        // Velocity PID's
                        new PIDController(Constants.DriveConstants.kPFrontLeftVel, 0, 0),
                        new PIDController(Constants.DriveConstants.kPRearLeftVel, 0, 0),
                        new PIDController(Constants.DriveConstants.kPFrontRightVel, 0, 0),
                        new PIDController(Constants.DriveConstants.kPRearRightVel, 0, 0),
                        robotDrive::getCurrentWheelSpeeds,
                        robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                        robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        addCommands(
                mecanumControllerCommand,
                new InstantCommand(() -> {
                    robotDrive.drive(0.0, 0.0, 0.0, fieldRelative);
                })
        );
    }
}

