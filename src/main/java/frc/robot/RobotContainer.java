// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousConePlacementCommand;
import frc.robot.commands.AutonomousFollowTrajectoryChargingCommand;
import frc.robot.commands.BalanceOnStationCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
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
    private final DrivetrainSubsystem mRobotDrive = new DrivetrainSubsystem();

    // The driver's controller
    private final Controller mController = new Controller(RobotMap.XBOX_CONTROLLER_ID);

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
     */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        mRobotDrive.setDefaultCommand(
                new RunCommand(
                        () -> mRobotDrive.driveRobotOriented(
                                mController.getThresholdedLeftX(),
                                mController.getThresholdedLeftY(),
                                mController.getThresholdedRightX()),
                        mRobotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one
     * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(mController, Button.kB.value)
                .onTrue(new BalanceOnStationCommand(mRobotDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new AutonomousConePlacementCommand(mRobotDrive), // Place the cone that robot holds
                new AutonomousFollowTrajectoryChargingCommand(mRobotDrive), // Follow trajectory to balancing station
                new BalanceOnStationCommand(mRobotDrive)); // Balance the robot on the station
    }
}
