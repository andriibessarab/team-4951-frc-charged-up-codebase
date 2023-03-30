// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.vision.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.drivetrain.AutoDrivePathPlannerTrajectory;
import frc.robot.commands.drivetrain.MecanumDriveExample;
import frc.robot.commands.vision.WatchForAprilTagPose;
import frc.robot.helpers.PathPlannerPath;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
        private final ArmSubsystem m_arm = new ArmSubsystem();
        private final PivotSubsystem m_pivot = new PivotSubsystem();
        private final LimelightSubsystem m_limeLight = new LimelightSubsystem(
                        Constants.LimelightSubsystem.kLimelightName);

        // The controllers
        XboxController m_driverController = new XboxController(OIConstants.DriverControl.kDriverControllerPort);
        XboxController m_operatorController = new XboxController(OIConstants.OperatorControl.kOperatorControllerPort);

        // Path planner trajectories
        PathPlannerPath[] m_pathPlannerPaths = {
                        new PathPlannerPath("basictest", true, 0.3, 0.3),
                        new PathPlannerPath("rotation", true, 0.3, 0.3),
        };

        SendableChooser m_autonomousOperation = new SendableChooser();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureButtonBindings();
                registerAutonomousOperations();

                m_limeLight.setDefaultCommand(new WatchForAprilTagPose(m_limeLight, m_robotDrive));

                // // Configure default commands set the default drive command to split-stick
                // arcade drive
                m_robotDrive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                new RunCommand(
                                                () -> {
                                                        var controllerLeftY = m_driverController.getLeftY()
                                                                        + Constants.OIConstants.DriverControl.kZeroCalibrateLeftY;
                                                        var controllerRightY = m_driverController.getRightX()
                                                                        + Constants.OIConstants.DriverControl.kZeroCalibrateRightX;
                                                        var controllerLeftX = m_driverController.getLeftX()
                                                                        + Constants.OIConstants.DriverControl.kZeroCalibrateLeftX;
                                                        m_robotDrive.drive(
                                                                        MathUtil.applyDeadband(-controllerLeftY,
                                                                                        Constants.OIConstants.DriverControl.kDriveDeadband),
                                                                        MathUtil.applyDeadband(-controllerRightY,
                                                                                        Constants.OIConstants.DriverControl.kDriveDeadband),
                                                                        MathUtil.applyDeadband(-controllerLeftX,
                                                                                        Constants.OIConstants.DriverControl.kRotationDeadband),
                                                                        false);
                                                },
                                                m_robotDrive));

                m_arm.setDefaultCommand(new RunCommand(
                                () -> {
                                        var controllerLeftX = m_operatorController.getLeftX()
                                                        + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
                                        m_elevator.setSpeed(controllerLeftX);
                                },
                                m_elevator));

                m_pivot.setDefaultCommand(new RunCommand(
                                () -> {
                                        var controllerRightY = m_operatorController.getRightY()
                                                        + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
                                        m_pivot.setSpeed(-controllerRightY);
                                },
                                m_pivot));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {

                ///////////////////////////////////////////////////////
                // DRIVER CONTROL
                ///////////////////////////////////////////////////////

                // Drive at half speed when the right bumper is held
                new JoystickButton(m_driverController, Button.kRightBumper.value)
                                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

                new JoystickButton(m_driverController, Button.kLeftBumper.value) // PS4 top left upper
                                .whenHeld(new AutoAlignWithRetroTape(m_limeLight, m_robotDrive));

                new JoystickButton(m_driverController, Button.kBack.value) // PS4 top left lower
                                .whenHeld(new AutoAlignWithAprilTag(m_limeLight, m_robotDrive));

                new JoystickButton(m_driverController, Button.kA.value) // PS4 kSquare
                                .whenHeld(new InstantCommand(m_robotDrive::updateSmartDashboard));

                new JoystickButton(m_driverController, Button.kY.value) // PS4 kTriangle
                                .whenHeld(new InstantCommand(m_robotDrive::zeroHeading));

                ///////////////////////////////////////////////////////
                // OPERATOR CONTROL
                ///////////////////////////////////////////////////////
                new JoystickButton(m_operatorController, Button.kY.value) // Xbox kY
                                .onTrue(new ElevatorGotoPosition(m_elevator,
                                                Constants.ElevatorSubsystem.kTopLevelHeight));

                new JoystickButton(m_operatorController, Button.kX.value) // Xbox kX
                                .onTrue(new ElevatorGotoPosition(m_elevator,
                                                Constants.ElevatorSubsystem.kMidLayerHeight));

                new JoystickButton(m_operatorController, Button.kA.value) // Xbox kA
                                .onTrue(new ElevatorGotoPosition(m_elevator,
                                                Constants.ElevatorSubsystem.kBottomLayerHeight));

                new JoystickButton(m_operatorController, Button.kB.value) // Xbox kB
                                .onTrue(new ArmGoToPosition(m_arm,
                                                m_arm.getPosition() > 0.5 ? Constants.ArmSubsystem.kMinExtend
                                                                : Constants.ArmSubsystem.kMaxExtend));

                new JoystickButton(m_operatorController, Button.kLeftBumper.value) // Xbox kLeftBumper
                                .onTrue(new PivotGoToPosition(m_pivot,
                                                m_pivot.getPosition() > 0.5 ? Constants.PivotSubsystem.kMinOut
                                                                : Constants.PivotSubsystem.kMaxOut));

        }

        private void registerAutonomousOperations() {
                m_autonomousOperation.setDefaultOption("Do Nothing",
                                new InstantCommand(() -> {
                                        m_robotDrive.drive(0.0, 0.0, 0.0, false);
                                }));

                for (int index = 0; index < m_pathPlannerPaths.length; index++) {
                        AutoDrivePathPlannerTrajectory drivePath = new AutoDrivePathPlannerTrajectory(m_robotDrive,
                                        m_pathPlannerPaths[index].name,
                                        m_pathPlannerPaths[index].resetOdometry,
                                        m_pathPlannerPaths[index].maxVelocity,
                                        m_pathPlannerPaths[index].maxAcceleration);
                        m_autonomousOperation.addOption(m_pathPlannerPaths[index].name, drivePath);
                }

                m_autonomousOperation.addOption("Mecanum Drive Example",
                                new MecanumDriveExample(m_robotDrive, true));

                SmartDashboard.putData("Autonomous Operation", m_autonomousOperation);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return (Command) m_autonomousOperation.getSelected();
        }

        public void setInitialAlliancePosition(double xPosition, double yPosition, int orientation) {
                m_robotDrive.resetOdometry(new Pose2d(xPosition, yPosition, new Rotation2d(orientation * 2 * Math.PI)));
        }
}
