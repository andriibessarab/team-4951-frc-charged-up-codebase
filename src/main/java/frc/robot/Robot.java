package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.subsystems.DrivetrainSubsystem;


/**
 * The Robot class is the main class of the robot program that extends the TimedRobot class.
 * It controls the overall behavior of the robot during different phases of operation such as
 * initialization, autonomous mode, teleoperated mode, test mode and simulation mode. It
 * also contains the code for the drivetrain subsystem that controls the robot's movement.
 * @author Andrii Bessarab
 * @author Sitong Li
 */
public class Robot extends TimedRobot {

    private DrivetrainSubsystem drivetrain;

    /**
     * This method is called once when the robot is first powered on. It sets the rear right
     * motor to be inverted.
     */
    @Override
    public void robotInit() {
        drivetrain = new DrivetrainSubsystem();
    }


    /**
     * This method is called periodically during operation.
     */
    @Override
    public void robotPeriodic() {
        drivetrain.updateOdometry();
    }


    /**
     * This method is called once when autonomous mode is first started.
     */
    @Override
    public void autonomousInit() {}


    /**
     * This method is called periodically during autonomous mode but does not do anything in this program.
     */
    @Override
    public void autonomousPeriodic() {}


    /**
     * This method is called once when teleoperated mode is first started.
     */
    @Override
    public void teleopInit() {}


    /**
     * This method is called periodically during teleoperated mode. It uses the values of the
     * left and right sticks on the controller to drive the robot oriented in the direction
     * indicated by the sticks.
     */
    @Override
    public void teleopPeriodic() {
        drivetrain.driveRobotOriented(
                RobotMap.mController1.getThresholdedLeftX(),
                RobotMap.mController1.getThresholdedLeftY(),
                RobotMap.mController1.getThresholdedRightX()
        );
    }


    /**
     * This method is called once when teleoperated mode is disabled.
     */
    @Override
    public void disabledInit() {}


    /**
     * This method is called periodically when the robot is disabled but does not do anything in this program.
     */
    @Override
    public void disabledPeriodic() {}


    /**
     * This method is called once when test mode is first started.
     */
    @Override
    public void testInit() {}


    /**
     * This method is called periodically during test mode but does not do anything in this program.
     */
    @Override
    public void testPeriodic() {}


    /**
     * This method is called once when simulation mode is first started.
     */
    @Override
    public void simulationInit() {}


    /**
     * This method is called periodically during simulation mode but does not do anything in this program.
     */
    @Override
    public void simulationPeriodic() {}
}
