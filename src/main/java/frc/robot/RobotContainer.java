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
import frc.robot.subsystems.LimelightVision;
import frc.robot.utils.Camera;
import frc.robot.utils.Controller;
import frc.robot.utils.Led;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 * @author Andrii Bessarab
 * @author Sitong Li
 */
public class RobotContainer {
    // The robot's subsystems
    private final DrivetrainSubsystem m_robotDrive = new DrivetrainSubsystem();

    // The driver's controller
    private final Controller mController1 = new Controller(RobotMap.XBOX_CONTROLLER_1_ID);
    private final Controller mController2 = new Controller(RobotMap.XBOX_CONTROLLER_2_ID);

    private final LimelightVision limelight = new LimelightVision(RobotMap.LIMELIGHT_HOSTNAME);
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final Camera frontCam = new Camera(RobotMap.CAMERA_FRONT_DEV, RobotMap.CAMERA_RES_W, RobotMap.CAMERA_RES_H);
    private final Camera intakeCam = new Camera(RobotMap.CAMERA_INTAKE_DEV, RobotMap.CAMERA_RES_W,
            RobotMap.CAMERA_RES_H);;
    private final Led led = new Led(RobotMap.LED_CHANNEL);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.driveRobotOriented(
                                mController1.getThresholdedLeftX(),
                                mController1.getThresholdedLeftY(),
                                mController1.getThresholdedRightX()),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one 
     * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or 
     * {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // if(mController1.getBButtonPressed())
        new JoystickButton(mController1, Button.kB.value)
                .onTrue(new BalanceOnStationCommand(drivetrain));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new AutonomousConePlacementCommand(drivetrain),
                new AutonomousFollowTrajectoryChargingCommand(drivetrain),
                new BalanceOnStationCommand(drivetrain));
    }
}
