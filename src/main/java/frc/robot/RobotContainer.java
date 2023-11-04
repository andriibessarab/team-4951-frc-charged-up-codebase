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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Declare robot subsystems
    public final Subsys_Drive m_robotDrive = new Subsys_Drive();
    private final Subsys_Elevator m_elevator = new Subsys_Elevator();
    private final Subsys_Arm m_arm = new Subsys_Arm();
    private final Subsys_Pivot m_pivot = new Subsys_Pivot();
    private final Subsys_Claw m_reach = new Subsys_Claw();
    private final Subsys_Intake m_claw = new Subsys_Intake();
    private final Subsys_Gyro m_gyro = new Subsys_Gyro();
    // private final LimelightSubsystem m_limeLight = new LimelightSubsystem(Constants.LimelightSubsystem.kLimelightName);

    // Declare input controllers
    XboxController m_driverController = new XboxController(Constants.OIConstants.DriverControl.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(Constants.OIConstants.OperatorControl.kOperatorControllerPort);
    // XboxController m_backupOperatorController = new XboxController(2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure default commands
        configureDefaultCommands();

        // Configure button bindings
        configureButtonBindings();
    }

    /**
     * Set default command for robot's subsystems.
     */
    private void configureDefaultCommands() {
        // Input operated sriving method
        m_robotDrive.setDefaultCommand(new InstantCommand(()-> {
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
                }, m_robotDrive));
        
        // Pivot set to constant speed if input provided
        // m_pivot.setDefaultCommand(new RunCommand(()-> {
        //         var controllerLeftY = m_backupOperatorController.getLeftY() +
        //             Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
        //         if (Math.abs(controllerLeftY) > 0.2) {
        //             m_pivot.setSpeed((Math.abs(controllerLeftY) / -controllerLeftY) * 0.2);
        //         } else {
        //             m_pivot.setSpeed(0);
        //         }
        //     }, m_pivot));
        
        // // Arm set to constant speed if input provided
        // m_arm.setDefaultCommand(new RunCommand(()-> {
        //         var controllerRightY = m_backupOperatorController.getRightY();
        //         if (Math.abs(controllerRightY) > 0.2) {
        //             m_arm.setSpeed((Math.abs(controllerRightY) / -controllerRightY) * 0.4);
        //         } else {
        //             m_arm.setSpeed(0);
        //         }
        //     }, m_arm));

        // Pivot set to constant speed if input provided
        m_pivot.setDefaultCommand(new RunCommand(()-> {
            var controllerLeftY = m_operatorController.getLeftY() +
                Constants.OIConstants.OperatorControl.kZeroCalibrateLeftY;
            if (Math.abs(controllerLeftY) > 0.2) {
                m_pivot.setSpeed((Math.abs(controllerLeftY) / -controllerLeftY) * 0.2);
            } else {
                m_pivot.setSpeed(0);
            }
        }, m_pivot));

        // Arm set to constant speed if input provided
        m_arm.setDefaultCommand(new RunCommand(()-> {
            var controllerRightY = m_operatorController.getRightY();
            if (Math.abs(controllerRightY) > 0.2) {
                m_arm.setSpeed((Math.abs(controllerRightY) / -controllerRightY) * 0.4);
            } else {
                m_arm.setSpeed(0);
            }
        }, m_arm));
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
        // Top shoot
        new JoystickButton(m_operatorController, Button.kY.value).onTrue(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kTopLayerHeight),
                        new CmdHybridManip_PivotGoToPosition(m_pivot, 2.3)
                    ),
                    new CmdHybridManipTimed_Outtake(m_claw, 0.6),
                    new ParallelCommandGroup(
                        new CmdHybridManip_PivotGoToPosition(m_pivot, 0.2),
                        new CmdHybridManip_ElevatorGoToPosition(m_elevator, 0)
                    )
        
                )
        );

        // Middle shoot
        new JoystickButton(m_operatorController, Button.kB.value).onTrue(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kMidLayerHeight),
                        new CmdHybridManip_PivotGoToPosition(m_pivot, 2.3)
                    ),
                    new CmdHybridManipTimed_Outtake(m_claw, 0.2),
                    new ParallelCommandGroup(
                        new CmdHybridManip_PivotGoToPosition(m_pivot, 0.2),
                        new CmdHybridManip_ElevatorGoToPosition(m_elevator, 0)
                    )
        
                )
        );

        // Low out
        new JoystickButton(m_operatorController, Button.kA.value).onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kBottomLayerHeight),
                    new CmdHybridManip_PivotGoToPosition(m_pivot, 3.6) // was 2.3
                )
            )
        );

        // Low in
        new JoystickButton(m_operatorController, Button.kX.value).onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kBottomLayerHeight),
                    new CmdHybridManip_PivotGoToPosition(m_pivot, 0.2) // was 2.3
                )
            )
        );

        // Home position
        // new JoystickButton(m_operatorController, Button.kX.value).onTrue(
        //         new SequentialCommandGroup(
        //             new ParallelCommandGroup(
        //                 new CmdHybridManip_PivotGoToPosition(m_pivot, 0),
        //                 new CmdHybridManip_ElevatorGoToPosition(m_elevator, 0)
        //             )
        //         )
        // );

        // Intake spin out
        new JoystickButton(m_operatorController, Button.kRightBumper.value) 
            .onTrue(new CmdHybridManipTimed_Outtake(m_claw, 0.5));
    
        // Intake spin in
        new JoystickButton(m_operatorController, Button.kLeftBumper.value)
            .whileHeld(new CmdHybridManip_Intake(m_claw));


        // // Elevator mid position    
        // new JoystickButton(m_backupOperatorController, Button.kB.value)
        //     .onTrue(new InstantCommand(()-> m_elevator.setPosition(Constants.ElevatorSubsystemConstants.kMidLayerHeight)));

        // // Elevator high position
        // new JoystickButton(m_backupOperatorController, Button.kY.value)
        //     .onTrue(new InstantCommand(()-> m_elevator.setPosition(Constants.ElevatorSubsystemConstants.kTopLayerHeight)));

        // // Elevator low position
        // new JoystickButton(m_backupOperatorController, Button.kA.value)
        //     .onTrue(new InstantCommand(()-> m_elevator.setPosition(Constants.ElevatorSubsystemConstants.kBottomLayerHeight)));

        // // Intake spin out
        // new JoystickButton(m_backupOperatorController, Button.kRightBumper.value) 
        //     .onTrue(new CmdHybridManipTimed_Outtake(m_claw, 0.75));

        // // Intake spin in
        // new JoystickButton(m_backupOperatorController, Button.kLeftBumper.value)
        //     .whileHeld(new CmdHybridManip_Intake(m_claw));

        // // Claw switch state of solenoid
        // new JoystickButton(m_backupOperatorController, Button.kRightStick.value) // FLIP PNEUMATICS STATE
        //     .onTrue(new InstantCommand(()-> m_reach.use()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new CmdHybridManip_ElevatorGoToPosition(m_elevator, Constants.ElevatorSubsystemConstants.kTopLayerHeight),
                new CmdHybridManip_PivotGoToPosition(m_pivot, 2.3)
            ),
            new CmdHybridManipTimed_Outtake(m_claw, 0.6),
            new ParallelCommandGroup(
                new CmdAutonDrive_LeaveCommZone(m_robotDrive),
                new CmdHybridManip_PivotGoToPosition(m_pivot, 0),
                new CmdHybridManip_ElevatorGoToPosition(m_elevator, 0)
            )

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
