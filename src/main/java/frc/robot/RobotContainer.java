// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.autonomous_commands.LeaveCommunityZone;
import frc.robot.commands.drivetrain_commands.AutoSQ_NewBalance;
import frc.robot.commands.intake_commands.*;
import frc.robot.commands.vision_commands.*;
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
        private final LimelightSubsystem m_limeLight = new LimelightSubsystem(
                        Constants.LimelightSubsystem.kLimelightName);

        // Operator input controllers
        XboxController m_driverController = new XboxController(OIConstants.DriverControl.kDriverControllerPort);
        XboxController m_operatorController = new XboxController(OIConstants.OperatorControl.kOperatorControllerPort);

        private final HttpCamera m_limelightCamera;

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
                m_limelightCamera = new HttpCamera("LimelightCamera", "http://10.49.51.11:1181/stream.mjpg");
                m_limelightCamera.setResolution(400, 400);
                CameraServer.addCamera(m_limelightCamera);
                Shuffleboard.getTab("Cameras").add(m_limelightCamera);
                UsbCamera cam = CameraServer.startAutomaticCapture(0);
                cam.setResolution(400, 400);
                // CameraServer.addCamera(cam);
                Shuffleboard.getTab("Cameras").add(cam);  


                configureButtonBindings();
                //registerAutonomousOperations();
                m_limeLight.setDefaultCommand(new WatchForAprilTagPose(m_limeLight, m_robotDrive));

                // // Configure default commands set the default drive command to split-stick
                // arcade drive
                m_robotDrive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                new InstantCommand(
                                                () -> {                                                    
                                                        var controllerLeftX = m_driverController.getLeftX();
                                                        var controllerLeftY = m_driverController.getLeftY();
                                                        var controllerRightX = m_driverController.getRightX();

                                                        m_robotDrive.driveMecanum(
                                                                        MathUtil.applyDeadband(controllerLeftX,
                                                                                        Constants.OIConstants.DriverControl.kDriveDeadband),
                                                                        MathUtil.applyDeadband(-controllerLeftY,
                                                                                        Constants.OIConstants.DriverControl.kDriveDeadband),
                                                                        MathUtil.applyDeadband(controllerRightX * 0.67,
                                                                                        Constants.OIConstants.DriverControl.kRotationDeadband)
                                                        );
                                                },
                                                m_robotDrive));

                // #TODO only for testing
                // m_arm.setDefaultCommand(new RunCommand(
                //                 () -> {
                //                         var controllerLeftY = m_operatorController.getLeftY()
                //                                         + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
                //                         m_arm.setSpeed(controllerLeftY);
                //                 },
                //                 m_arm));

                // #TODO only for testing
                m_pivot.setDefaultCommand(new RunCommand(
                                () -> {
                                        var controllerLeftY = m_operatorController.getLeftY()
                                                        + Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
                                        if(Math.abs(controllerLeftY)>0.2){
                                                m_pivot.setSpeed((Math.abs(controllerLeftY)/-controllerLeftY) * 0.2);
                                        } else{
                                                m_pivot.setSpeed(0);
                                        }
                                },
                                m_pivot));
                
                //TODO: test to see if work, might keep for actual use
                // m_claw.setDefaultCommand(new Cmd_defaultClawCommand(m_claw, m_driverController));

                
                

                // TODO: will override the command above, testing only, allow command above to stay
                // m_claw.setDefaultCommand(new RunCommand(
                //                 () -> {
                //                         var rTrigger = m_operatorController.getRightTriggerAxis();
                //                         var lTrigger = m_operatorController.getLeftTriggerAxis();
                //                         if(rTrigger>0.2){
                //                                 new ClawOutake(m_claw);
                //                         } else if(lTrigger>0.2){
                //                                 new ClawIntake(m_claw);
                //                         }
                //                 },
                //                 m_claw));
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
                // new JoystickButton(m_driverController, Button.kRightBumper.value)// Xbox kRightBumper
                //                 .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                //                 .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

                new JoystickButton(m_driverController, Button.kA.value)
                                .onTrue(new AutoSQ_NewBalance(0, m_robotDrive));

                // Update smart dashboard values
                // new JoystickButton(m_driverController, Button.kLeftBumper.value) // Xbox kA
                //                 .whenHeld(new RepeatCommand(new InstantCommand(m_robotDrive::strafeLeft)));

                // // Reset gyro heading to zero
                // new JoystickButton(m_driverController, Button.kRightBumper.value) // Xbox kY
                //                 .whenHeld(new RepeatCommand(new InstantCommand(m_robotDrive::strafeRight)));

                // // Update smart dashboard values
                // new JoystickButton(m_driverController, Button.kX.value) // Xbox kA
                //                 .whenHeld(new RepeatCommand(new InstantCommand(()->{m_robotDrive.strafeLeftQuarter();})));
                
                // // Reset gyro heading to zero
                // new JoystickButton(m_driverController, Button.kB.value) // Xbox kY
                //                 .whenHeld(new RepeatCommand(new InstantCommand(()->{m_robotDrive.strafeLeftQuarter();})));


                ///////////////////////////////////////////////////////
                // VISION
                ///////////////////////////////////////////////////////

                // // Align robot with detected retro tape
                // new JoystickButton(m_driverController, Button.kLeftBumper.value) // Xbox kLeftBumper
                //                 .whenHeld(new AutoAlignWithRetroTape(m_limeLight, m_robotDrive));

                // // Align robot with detected april tag
                // new JoystickButton(m_driverController, Button.kX.value) // Xbox kX
                //                 .whenHeld(new AutoAlignWithAprilTag(m_limeLight, m_robotDrive));

                ///////////////////////////////////////////////////////
                // DRIVE MISC
                ///////////////////////////////////////////////////////

                // Update smart dashboard values
                // new JoystickButton(m_driverController, Button.kA.value) // Xbox kA
                //                 .whenHeld(new InstantCommand(m_robotDrive::updateSmartDashboard));

                // // Reset gyro heading to zero
                // new JoystickButton(m_driverController, Button.kY.value) // Xbox kY
                //                 .whenHeld(new InstantCommand(m_robotDrive::zeroHeading));
                

                ///////////////////////////////////
                // OPERATOR CONTROLS             //
                //                               //
                //  ELEVATOR                     //
                //    Button Y - Go to Top Level //
                //    button B - Go to Mid Level //
                //    Button A - Go to Low Level //
                //                               //
                //  ARM                          //
                //    [to be determined]         //
                //                               //
                //  PIVOT                        //
                //    Left Joystick Y - Manual   //
                //    Button X - Pivot In        //
                //    Button Start - Pivot Out   //
                ///////////////////////////////////

                ///////////////////////////////////////////////////////
                // ELEVATOR
                ///////////////////////////////////////////////////////

                new JoystickButton(m_operatorController, Button.kY.value)       // HIGH ELEVATOR POS
                        .onTrue(new InstantCommand(() -> m_elevator.setPosition( Constants.ElevatorSubsystem.kTopLayerHeight) ));
                
                new JoystickButton(m_operatorController, Button.kB.value)       // MID ELEVATOR POS
                        .onTrue(new InstantCommand(() -> m_elevator.setPosition( Constants.ElevatorSubsystem.kMidLayerHeight) ));

                new JoystickButton(m_operatorController, Button.kA.value)       // LOW ELEVATOR POS
                        .onTrue(new InstantCommand(() -> m_elevator.setPosition( Constants.ElevatorSubsystem.kBottomLayerHeight ) ));

                ///////////////////////////////////////////////////////
                // PIVOT
                ///////////////////////////////////////////////////////

                // new JoystickButton(m_operatorController, Button.kX.value) // PIVOT IN
                //         .onTrue(new PivotGoToPosition(m_pivot, 0.1));


                // new JoystickButton(m_operatorController, Button.kStart.value) // PIVOT OUT
                //         .onTrue(new PivotGoToPosition(m_pivot, 3.6));

                ///////////////////////////////////////////////////////
                // INTAKE
                ///////////////////////////////////////////////////////

                new JoystickButton(m_operatorController, Button.kRightBumper.value) // INTAKE OUT
                        .onTrue(new ClawOutake(m_claw, 0.75));

                new JoystickButton(m_operatorController, Button.kLeftBumper.value) // INTAKE IN
                        .whileHeld(new ClawIntake(m_claw));

                ///////////////////////////////////////////////////////
                // CLAW PNEUMATICS
                ///////////////////////////////////////////////////////

                new JoystickButton(m_operatorController, Button.kRightStick.value) // FLIP PNEUMATICS STATE
                        .onTrue(new InstantCommand(()->m_reach.use())); 
        }

        // /**
        //  * Registers the available autonomous operations that the robot can perform during autonomous mode.
        //  * This method populates the m_autonomousOperation object with available autonomous options.
        //  */
        // ArrayList<AutoDrivePathPlannerTrajectory> paths = new ArrayList<>();
        // private void registerAutonomousOperations() {
        //         m_autonomousOperation.setDefaultOption("Do Nothing",
        //                         new InstantCommand(() -> {
        //                                 m_robotDrive.drive(0.0, 0.0, 0.0, false);
        //                         }));

        //         for (int index = 0; index < m_pathPlannerPaths.length; index++) {
        //                 AutoDrivePathPlannerTrajectory drivePath = new AutoDrivePathPlannerTrajectory(m_robotDrive,
        //                                 m_pathPlannerPaths[index].name,
        //                                 m_pathPlannerPaths[index].resetOdometry,
        //                                 m_pathPlannerPaths[index].maxVelocity,
        //                                 m_pathPlannerPaths[index].maxAcceleration);
        //                 paths.add(drivePath);
        //                 m_autonomousOperation.addOption(m_pathPlannerPaths[index].name, drivePath);
        //         }

        //         m_autonomousOperation.addOption("Mecanum Drive Example",
        //                         new MecanumDriveExample(m_robotDrive, true));

        //         SmartDashboard.putData("Autonomous Operation", m_autonomousOperation);
        // }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        // public Command getAutonomousCommand() {
        //         return new SequentialCommandGroup(
        //                 new BasicAutonomous(m_robotDrive),
        //                 new InstantCommand(()->m_robotDrive.driveMecanum(0, 0, 0))
        //         );
        // }

        public Command getAutonomousCommand() {
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ElevatorGotoPosition(m_elevator, Constants.ElevatorSubsystem.kTopLayerHeight),
                                new PivotGoToPositionAuto(m_pivot, 2.3)
                        ),
                        new ClawOutake(m_claw, 0.6),
                        new ParallelCommandGroup(
                                new LeaveCommunityZone(m_robotDrive),
                                new PivotGoToPositionAuto(m_pivot, 0),
                                new ElevatorGotoPosition(m_elevator, 0)
                        )

                );
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */


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
