package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Subsystems
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// Utils
import frc.robot.utils.TimerUtil;
import frc.robot.utils.Gyroscope;
import frc.robot.utils.Motors;
import frc.robot.utils.Controller;


public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        // Set up drivetrain
        Motors.initialize();
        Motors.setSpeed(0.5);

        // Calibrate gyroscope
        Gyroscope.calibrate();
    }


    @Override
    public void robotPeriodic() {
        // DEBUG - Update smart dashboard periodically
        SmartDashboard.putBoolean("Limelight Has Target", Limelight.hasTarget());
        SmartDashboard.putNumber("Limelight Horizontal Offset", Limelight.getHorizontalOffset());
        SmartDashboard.putNumber("Limelight Vertical Offset", Limelight.getVerticalOffset());
        SmartDashboard.putNumber("Limelight Target Area", Limelight.getTargetArea());

    }


    @Override
    public void autonomousInit() {
        // Reset timer before starting autonomous
        TimerUtil.reset();
        TimerUtil.start();
    }


    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {}


    @Override
    public void teleopPeriodic() {
        double x = 0; // x-axis motion-right (+), left (-)
        double y = 0; // y-axis motion-forward (+), backward (-)
        double z = 0; // z-axis motion-clockwise (+), counterclockwise (-)

        /* Pushing the right stick on it’s x-axis give a x-value (+ = right, - = left)
         * Pushing the left stick on it’s y-axis gives a y-value (+ = forward, - = backward)
         * Pushing the left stick on it’s x-axis gives a z-value (+ = clockwise, - = backward)
         */
        Drivetrain.drive(
            Controller.getRightX(true),
            Controller.getLeftY(true),
            Controller.getLeftX(true)
        );
    }


    @Override
    public void disabledInit() {}


    @Override
    public void disabledPeriodic() {}


    @Override
    public void testInit() {}


    @Override
    public void testPeriodic() {}


    @Override
    public void simulationInit() {}


    @Override
    public void simulationPeriodic() {}

}
