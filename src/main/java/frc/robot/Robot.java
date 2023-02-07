package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Subsystems
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// Utils
import frc.robot.utils.TimerUtil;
import frc.robot.utils.Gyroscope;
import frc.robot.utils.Controller;


public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        // Set up drivetrain
        Drivetrain.initializeMotors();
        Drivetrain.setSpeed(1);

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
        // Drive robot
        Drivetrain.drive(
          Controller.getLeftX(),
          Controller.getLeftY(),
          Controller.getRightX(),
          Gyroscope.getAngle()
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
