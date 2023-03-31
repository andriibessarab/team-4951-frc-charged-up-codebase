// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.drivetrain_commands.AutoDrivePathPlannerTrajectory;
import frc.robot.commands.drivetrain_commands.MecanumDriveExample;
import frc.robot.commands.intake_commands.*;
import frc.robot.commands.vision_commands.*;
import frc.robot.helpers.PathPlannerPath;
import frc.robot.subsystems.drivetrain_subsystems.*;
import frc.robot.subsystems.intake_subsystems.*;
import frc.robot.subsystems.vision_subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Robot's dirvetrain subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        // Robot's intake subsystems
        private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
        private final ArmSubsystem m_arm = new ArmSubsystem();
        private final PivotSubsystem m_pivot = new PivotSubsystem();
        private final ClawPneumaticsSubsystem m_reach = new ClawPneumaticsSubsystem();
        private final ClawMotorsSubsystem m_claw = new ClawMotorsSubsystem();

        // Robot's vision subsystems
        private final LimelightSubsystem m_limeLight = new LimelightSubsystem(
                        Constants.LimelightSubsystem.kLimelightName);

        // Operator input controllers
        XboxController m_driverController = new XboxController(OIConstants.DriverControl.kDriverControllerPort);
        XboxController m_operatorController = new XboxController(OIConstants.OperatorControl.kOperatorControllerPort);

        // Path planner trajectories
        PathPlannerPath[] m_pathPlannerPaths = {
                        new PathPlannerPath("openSidePreload1", true, 0.4, 0.3),
                        new PathPlannerPath("openSidePreload2", true, 0.4, 0.3),
                        new PathPlannerPath("basictest", true, 0.3, 0.3),
                        new PathPlannerPath("rotation", true, 0.3, 0.3),
        };

        // Autonomous command selecter
        SendableChooser m_autonomousOperation = new SendableChooser();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                configureButtonBindings();
                //registerAutonomousOperations();

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
                                // new RunCommand(
                                //         ()->{
                                //                 var controllerLeftY = m_driverController.getLeftY()
                                //                                         + Constants.OIConstants.DriverControl.kZeroCalibrateLeftY;
                                //                         var controllerRightY = m_driverController.getRightX()
                                //                                         + Constants.OIConstants.DriverControl.kZeroCalibrateRightX;
                                //                         var controllerLeftX = m_driverController.getLeftX()
                                //                                         + Constants.OIConstants.DriverControl.kZeroCalibrateLeftX;
                                //                         m_robotDrive.driveMecanum(
                                //                                         MathUtil.applyDeadband(-controllerLeftY,
                                //                                                         Constants.OIConstants.DriverControl.kDriveDeadband),
                                //                                         MathUtil.applyDeadband(-controllerRightY,
                                //                                                         Constants.OIConstants.DriverControl.kDriveDeadband),
                                //                                         MathUtil.applyDeadband(-controllerLeftX,
                                //                                                         Constants.OIConstants.DriverControl.kRotationDeadband)
                                //                         );
                                //         }
                                // );

                // #TODO only for testing
                m_arm.setDefaultCommand(new RunCommand(
                                () -> {
                                        var controllerLeftX = m_operatorController.getLeftX()
                                                        + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
                                        m_arm.setSpeed(controllerLeftX);
                                },
                                m_arm));

                // #TODO only for testing
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
                new JoystickButton(m_driverController, Button.kRightBumper.value)// Xbox kRightBumper
                                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

                // Align robot with detected retro tape
                new JoystickButton(m_driverController, Button.kLeftBumper.value) // Xbox kLeftBumper
                                .whenHeld(new AutoAlignWithRetroTape(m_limeLight, m_robotDrive));

                // Align robot with detected april tag
                new JoystickButton(m_driverController, Button.kX.value) // Xbox kX
                                .whenHeld(new AutoAlignWithAprilTag(m_limeLight, m_robotDrive));

                // Update smart dashboard values
                new JoystickButton(m_driverController, Button.kA.value) // Xbox kA
                                .whenHeld(new InstantCommand(m_robotDrive::updateSmartDashboard));

                // Reset gyro heading to zero
                new JoystickButton(m_driverController, Button.kY.value) // Xbox kY
                                .whenHeld(new InstantCommand(m_robotDrive::zeroHeading));
                
                // #TODO add command for 180

                ///////////////////////////////////////////////////////
                // OPERATOR CONTROL
                ///////////////////////////////////////////////////////

                // Move elevator to top layer
                new JoystickButton(m_operatorController, Button.kY.value) // Xbox kY
                                .onTrue(new ElevatorGotoPosition(m_elevator,
                                                Constants.ElevatorSubsystem.kTopLayerHeight));

                // Move elevator to middle layer
                new JoystickButton(m_operatorController, Button.kX.value) // Xbox kX
                                .onTrue(new ElevatorGotoPosition(m_elevator,
                                                Constants.ElevatorSubsystem.kMidLayerHeight));

                // Move elevator to bottom layer
                new JoystickButton(m_operatorController, Button.kA.value) // Xbox kA
                                .onTrue(new ElevatorGotoPosition(m_elevator,
                                                Constants.ElevatorSubsystem.kBottomLayerHeight));

                // Extend arm if not, otherwise move in
                new JoystickButton(m_operatorController, Button.kB.value) // Xbox kB
                                .onTrue(new ArmGoToPosition(m_arm,
                                                m_arm.getPosition() > 0.5 ? Constants.ArmSubsystem.kMinExtend
                                                                : Constants.ArmSubsystem.kMaxExtend));

                // Open pivot if closed, otherwise close
                new JoystickButton(m_operatorController, Button.kLeftBumper.value) // Xbox kLeftBumper
                                .onTrue(new PivotGoToPosition(m_pivot,
                                                m_pivot.getPosition() > 0.5 ? Constants.PivotSubsystem.kMinOut
                                                                : Constants.PivotSubsystem.kMaxOut));

                new JoystickButton(m_operatorController, Button.kRightBumper.value)
                                .onTrue(new InstantCommand(()->m_reach.use()));
                if(m_operatorController.getRightTriggerAxis()>0.2){
                        m_claw.spinIn();
                } else if(m_operatorController.getLeftTriggerAxis()>0.2){
                        m_claw.spinOut();
                } else{
                        m_claw.stop();
                }

        }

        /**
         * Registers the available autonomous operations that the robot can perform during autonomous mode.
         * This method populates the m_autonomousOperation object with available autonomous options.
         */
        ArrayList<AutoDrivePathPlannerTrajectory> paths = new ArrayList<>();
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
                        paths.add(drivePath);
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
                AutoDrivePathPlannerTrajectory drivePath = new AutoDrivePathPlannerTrajectory(m_robotDrive,
                m_pathPlannerPaths[0].name,
                m_pathPlannerPaths[0].resetOdometry,
                m_pathPlannerPaths[0].maxVelocity,
                m_pathPlannerPaths[0].maxAcceleration);
                paths.add(drivePath);
                return new SequentialCommandGroup(
                        new ElevatorGotoPosition(m_elevator, Constants.ElevatorSubsystem.kTopLayerHeight),
                        new PivotGoToPosition(m_pivot, Constants.PivotSubsystem.kMaxOut),
                        new ArmGoToPosition(m_arm, Constants.ArmSubsystem.kMaxExtend),
                        new ClawOutake(m_claw),
                        drivePath,
                        new ElevatorGotoPosition(m_elevator, Constants.ElevatorSubsystem.kMinHeight),
                        new PivotGoToPosition(m_pivot, Constants.PivotSubsystem.kMinOut),
                        new ArmGoToPosition(m_arm, Constants.ArmSubsystem.kMinExtend)
                );
        }

        /**
         * Sets the initial position and orientation of the robot based on the alliance start position.
         *
         * @param xPosition  the x-coordinate of the robot's starting position
         * @param yPosition  the y-coordinate of the robot's starting position
         * @param orientation  the orientation of the robot, represented as an integer where 0 represents 0 degrees,
         *                     1 represents 90 degrees, 2 represents 180 degrees, and 3 represents 270 degrees.
         */
        public void setInitialAlliancePosition(double xPosition, double yPosition, int orientation) {
                m_robotDrive.resetOdometry(new Pose2d(xPosition, yPosition, new Rotation2d(orientation * 2 * Math.PI)));
        }
}
