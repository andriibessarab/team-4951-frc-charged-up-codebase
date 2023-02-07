package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PathWeaver;


public class Robot extends TimedRobot {
    // This will fix all your problems!!!
    public static final String Jason = "Jason wrote this line of code";
    
    // Xbox Controller
    private final XboxController mController = new XboxController(0);
    
    // Gyro
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    // Timer
    private final Timer mTimer = new Timer();

    // Variables
    //private double angle = 0; 


    @Override
    public void robotInit() {
        // Set up drivetrain
        Drivetrain.initializeMotors();
        Drivetrain.setSpeed(1);

        // Calibrate gyroscope
        gyro.calibrate();
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
        mTimer.reset();
        mTimer.start();
    }


    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {}


    @Override
    public void teleopPeriodic() {
        // Drive robot
        Drivetrain.drive(
          mController.getLeftX(),
          mController.getLeftY(),
          mController.getRightX(),
          gyro.getAngle()
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
