// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autonomous_commands.CmdSeqAuton_ElevatorPivotShootCubeDrive;
import frc.robot.commands.intake_commands.Cmd_ClawIntake;
import frc.robot.commands.intake_commands.Cmd_ClawOuttake;
import frc.robot.commands.intake_commands.Cmd_ArmGoToPosition;
import frc.robot.commands.intake_commands.Cmd_ElevatorGoToPosition;
import frc.robot.commands.intake_commands.Cmd_PivotGoToPosition;
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
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // Robot's intake subsystems
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();
    private final PivotSubsystem m_pivot = new PivotSubsystem();
    private final ClawPneumaticsSubsystem m_reach = new ClawPneumaticsSubsystem();
    private final ClawMotorsSubsystem m_claw = new ClawMotorsSubsystem();

    // Robot's vision subsystems
    private final LimelightSubsystem m_limeLight = new LimelightSubsystem(Constants.LimelightSubsystem.kLimelightName);

    // Operator input controllers
    XboxController m_driverController = new XboxController(OIConstants.DriverControl.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(OIConstants.OperatorControl.kOperatorControllerPort);

    // // Path planner trajectories
    // PathPlannerPath[] m_pathPlannerPaths = {
    //                 new PathPlannerPath("openSidePreload1", true, 0.4, 0.3),
    //                 new PathPlannerPath("openSidePreload2", true, 0.4, 0.3),
    //                 new PathPlannerPath("basictest", true, 0.3, 0.3),
    //                 new PathPlannerPath("rotation", true, 0.3, 0.3),
    // };

    // // Autonomous command selecter
    // SendableChooser m_autonomousOperation = new SendableChooser();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        configureButtonBindings();
        //registerAutonomousOperations();

        // // Configure default commands set the default drive command
        m_robotDrive.setDefaultCommand(new RunCommand(() -> {
            var controllerLeftX = m_driverController.getLeftX();
            var controllerLeftY = m_driverController.getLeftY();
            var controllerRightX = m_driverController.getRightX();

            m_robotDrive.drive(
                MathUtil.applyDeadband(-controllerLeftY, Constants.OIConstants.DriverControl.kDriveDeadband),
                MathUtil.applyDeadband(controllerLeftX, Constants.OIConstants.DriverControl.kDriveDeadband), 
                MathUtil.applyDeadband(controllerRightX * 0.67, Constants.OIConstants.DriverControl.kRotationDeadband),
                false
            );}, m_robotDrive));

        // #TODO only for testing
        // m_arm.setDefaultCommand(new RunCommand(() -> {
        //         var controllerLeftY = m_operatorController.getLeftY() + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
        //         m_arm.setSpeed(controllerLeftY);
        //     },
        //     m_arm));

        // // #TODO only for testing
        // m_pivot.setDefaultCommand(new RunCommand(() -> {
        //         var controllerRightY = m_operatorController.getRightY() + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
        //         m_pivot.setSpeed(-controllerRightY * 0.2);
        //     },
        //     m_pivot));

        //TODO: test to see if work, might keep for actual use
        // m_claw.setDefaultCommand(new RunCommand(
        //                 () -> {
        //                         var rTrigger = m_operatorController.getRightTriggerAxis();
        //                         var lTrigger = m_operatorController.getLeftTriggerAxis();
        //                         if(rTrigger>0.2){
        //                                 m_claw.spinIn();
        //                         } else if(lTrigger>0.2){
        //                                 m_claw.spinOut();
        //                         } else{
        //                                 m_claw.stop();
        //                         }
        //                 },
        //                 m_claw));

        //TODO: will override the command above, testing only, allow command above to stay
        m_elevator.setDefaultCommand(new RunCommand(() -> {
                var rTrigger = m_operatorController.getRightTriggerAxis();
                var lTrigger = m_operatorController.getLeftTriggerAxis();
                if (rTrigger > 0.2) {
                    m_elevator.setSpeed1(rTrigger / 3);
                } else if (lTrigger > 0.2) {
                    m_elevator.setSpeed1(-lTrigger / 3);
                } else {
                    m_elevator.stop();
                }
            },
            m_elevator));
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
        // DRIVERTRAIN
        ///////////////////////////////////////////////////////

        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kRightBumper.value) // Xbox kRightBumper
            .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5))).onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

        ///////////////////////////////////////////////////////
        // DRIVE MISC
        ///////////////////////////////////////////////////////

        // // Update smart dashboard values
        // new JoystickButton(m_driverController, Button.kA.value) // Xbox kA
        //     .whenHeld(new InstantCommand(m_robotDrive::updateSmartDashboard));

        // // Reset gyro heading to zero
        // new JoystickButton(m_driverController, Button.kY.value) // Xbox kY
        //     .whenHeld(new InstantCommand(m_robotDrive::zeroHeading));

        ///////////////////////////////////////////////////////
        // VISION
        ///////////////////////////////////////////////////////

        // Align robot with detected retro tape
        // new JoystickButton(m_driverController, Button.kLeftBumper.value) // Xbox kLeftBumper
        //     .whenHeld(new AutoAlignWithRetroTape(m_limeLight, m_robotDrive));

        // // Align robot with detected april tag
        // new JoystickButton(m_driverController, Button.kX.value) // Xbox kX
        //     .whenHeld(new AutoAlignWithAprilTag(m_limeLight, m_robotDrive));

        ///////////////////////////////////////////////////////
        // ELEVATOR
        ///////////////////////////////////////////////////////

        // Move elevator to top layer
        new JoystickButton(m_operatorController, Button.kY.value) // Xbox kY
            .onTrue(new Cmd_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kTopLayerHeight));

        // Move elevator to middle layer
        new JoystickButton(m_operatorController, Button.kB.value) // Xbox kB
            .onTrue(new Cmd_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kMidLayerHeight));

        // Move elevator to bottom layer
        new JoystickButton(m_operatorController, Button.kA.value) // Xbox kA
            .onTrue(new Cmd_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kBottomLayerHeight));

        ///////////////////////////////////////////////////////
        // ARM
        ///////////////////////////////////////////////////////

        // Retract arm
        new JoystickButton(m_operatorController, Button.kLeftStick.value) // Xbox kLeftStick
            .onTrue(new Cmd_ArmGoToPosition(m_arm, Constants.ArmSubsystemConstants.kMinExtend));

        // Extend arm
        new JoystickButton(m_operatorController, Button.kRightStick.value) // Xbox kRightStick
            .onTrue(new Cmd_ArmGoToPosition(m_arm, Constants.ArmSubsystemConstants.kMaxExtend));

        ///////////////////////////////////////////////////////
        // PIVOT
        ///////////////////////////////////////////////////////

        // Close pivot
        new JoystickButton(m_operatorController, Button.kLeftBumper.value) // Xbox kLeftBumper
            //.onTrue(new PivotOpen(m_pivot).andThen(()->m_pivot.stop()));
            .onTrue(new Cmd_PivotGoToPosition(m_pivot, 0));

        // Open pivot
        new JoystickButton(m_operatorController, Button.kRightBumper.value) // Xbox kRightBumper
            .onTrue(new Cmd_PivotGoToPosition(m_pivot, 1.7));

        ///////////////////////////////////////////////////////
        // CLAW
        ///////////////////////////////////////////////////////

        // Open claw if closed, otherwise close
        // new JoystickButton(m_operatorController, Button.kX.value) //pneumatic
        //     .onTrue(new InstantCommand(() -> m_reach.use()));

        // Spin claw motors inwards
        new JoystickButton(m_operatorController, Button.kBack.value) // Xbox back
            .onTrue(new Cmd_ClawIntake(m_claw));

        // Spin claw motors outwards
        new JoystickButton(m_operatorController, Button.kStart.value) // Xbox start
            .onTrue(new Cmd_ClawOuttake(m_claw));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     *
     */
    public Command getAutonomousCommand() {
        return new CmdSeqAuton_ElevatorPivotShootCubeDrive(m_robotDrive, m_elevator, m_pivot, m_claw);
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