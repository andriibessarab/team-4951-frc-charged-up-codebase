// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalanceOnStationCommand;
import frc.robot.subsystems.RollerIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.Controller;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 * 
 * @author Andrii Bessarab
 * @author Sitong Li
 */
public class RobotContainer {
    // The robot's subsystems
    private final SparkMaxDrivetrainSubsystem m_RobotDrive = new SparkMaxDrivetrainSubsystem();

    // The driver's controllers
    private final Controller m_DrivingController = new Controller(RobotMap.XBOX_DRIVER_CONTROLLER_ID);

    SendableChooser<Command> pathChooser = new SendableChooser<>();
    /*
     * Robot utils that currently not in use
     * private final Camera m_frontCam = new Camera(RobotMap.CAMERA_FRONT_DEV,
     * RobotMap.CAMERA_RES_W, RobotMap.CAMERA_RES_H);
     * private final Camera m_intakeCam = new Camera(RobotMap.CAMERA_INTAKE_DEV,
     * RobotMap.CAMERA_RES_W,
     * RobotMap.CAMERA_RES_H);;
     * private final Led m_led = new Led(RobotMap.LED_CHANNEL);
     * private final LimelightVision m_limelight = new
     * LimelightVision(RobotMap.LIMELIGHT_HOSTNAME);
     * private final ElevatorSubsystem mElevator = new ElevatorSubsystem();
     * private final RollerIntakeSubsystem mIntake = new RollerIntakeSubsystem();
     * private final ArmSubsystem mArm = new ArmSubsystem();
     */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        //configureButtonBindings();

        // Configure default commands
        m_RobotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_RobotDrive.driveMecanum(
                                m_DrivingController.getThresholdedLeftX(),
                                m_DrivingController.getThresholdedLeftY(),
                                m_DrivingController.getThresholdedRightX()),
                        m_RobotDrive));

        /*
         * pathChooser.addOption("curvy path",
         * loadPathplannerTrajectoryToRamseteCommand(
         * ".\\src\\main\\deploy\\pathplanner\\generatedJSON\\curvy.wpilib.json",
         * true));
         */
        Shuffleboard.getTab("Autonomous").add(pathChooser);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one
     * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_DrivingController, Button.kB.value)
                .onTrue(new BalanceOnStationCommand(m_RobotDrive));
    }



    /*
     * public Command loadPathplannerTrajectoryToRamseteCommand(String filename,
     * boolean resetOdometry) {
     * Trajectory trajectory;
     * 
     * try {
     * Path trajectoryPath =
     * Filesystem.getDeployDirectory().toPath().resolve(filename);
     * trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     * } catch (IOException exception) {
     * DriverStation.reportError("Unable to resolve trajectory " + filename,
     * exception.getStackTrace());
     * return new InstantCommand();
     * }
     * RamseteCommand ramseteCommand = new RamseteCommand(
     * trajectory,
     * mRobotDrive::getPose,
     * new RamseteController(
     * DrivetrainConstants.kRamseteB,
     * DrivetrainConstants.kRamseteZeta),
     * new SimpleMotorFeedforward(
     * DrivetrainConstants.ksVolts,
     * DrivetrainConstants.kvVoltSecondsPerMinuite,
     * DrivetrainConstants.kaVoltSecondsSquaredPerMinuite),
     * DrivetrainConstants.kDriveKinematics,
     * mRobotDrive::getWheelSpeeds,
     * new PIDController(
     * DrivetrainConstants.kpDriveVel,
     * 0,
     * 0),
     * new PIDController(
     * DrivetrainConstants.kpDriveVel,
     * 0,
     * 0),
     * mRobotDrive::setDriveVolts,
     * mRobotDrive);
     * if (resetOdometry) {
     * return new SequentialCommandGroup(
     * new InstantCommand(() ->
     * mRobotDrive.resetOdometry(trajectory.getInitialPose())), ramseteCommand);
     * } else {
     * return ramseteCommand;
     * }
     * }
     */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /*public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                // new AutonomousConePlacementCommand(mRobotDrive, mElevator, mIntake, mArm), //
                // Place the cone that robot holds
                // pathChooser.getSelected(), // Follow trajectory to balancing station
                new BalanceOnStationCommand(mRobotDrive) // Balance the robot on the station
        );
    }*/
}
