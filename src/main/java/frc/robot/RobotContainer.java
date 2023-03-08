// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Subsystems.DrivetrainSubsystem;
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
    private final DrivetrainSubsystem m_DrivetrainSubsystem = new DrivetrainSubsystem();

    // The driver's controllers
    private final Controller m_Controller = new Controller(RobotMap.XBOX_DRIVER_CONTROLLER_ID);

    // Shuffleboard elements
    SendableChooser<Command> pathChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureBindings();

        // Configure default commands
        m_DrivetrainSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> m_DrivetrainSubsystem.driveMecanum(
                                m_Controller.getThresholdedLeftX(),
                                m_Controller.getThresholdedLeftY(),
                                m_Controller.getThresholdedRightX()),
                        m_DrivetrainSubsystem));

        // Add paths to sendable chooser
        pathChooser.addOption("PathWithStrafing", m_DrivetrainSubsystem
                .followTrajectoryCommand(
                        PathPlanner.loadPath("PathWithStrafing", new PathConstraints(4, 3)), true));

        // Add shufflebord elements
        Shuffleboard.getTab("Autonomous").add(pathChooser);

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Balance on charging station when "B" is pressed
        /*
         * new JoystickButton(m_Controller, Button.kB.value)
         * .onTrue());
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                pathChooser.getSelected() // Selected autonomous path
        );
    }
}
