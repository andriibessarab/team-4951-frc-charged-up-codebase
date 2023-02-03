package frc.robot;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {
  // Xbox Controller
  private final XboxController m_controller = new XboxController(0);

  // Gyro
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Motors
  private final Victor rearLeftMotor = new Victor(0);
  private final Victor rearRightMotor = new Victor(1);
  private final Victor frontLeftMotor = new Victor(2);
  private final Victor frontRightMotor = new Victor(3);

  // Timer
  private final Timer m_timer = new Timer();

  // Variables
  private double SpeedMultiplier = 1; // 0 < motorSpeed <= 1 Change this to reduce overall speed of motorls

  @Override
  public void robotInit() {
    // Invert neccessary motors
    rearRightMotor.setInverted(true);

    // Set up gyro
    gyro.calibrate();
    gyro.reset();
  }


  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
    // Reset tier before starting autonomous
    m_timer.reset();
    m_timer.start();
  }


  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
    // Retrive axis from controller
    double y = -m_controller.getLeftY();
    double x = m_controller.getLeftX() * 1.1; // Counteract imperfect strafing
    double rx = m_controller.getRightX();

    double botHeading = gyro.getAngle();

    // Rotate the movement direction counter to the bot's rotation
    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

    // Calculate denominator
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

    // Calculate motor power
    double rearLeftPower = (y - rotX + rx) / denominator;
    double frontLeftPower = (y + rotX + rx) / denominator;
    double frontRightPower = (y - rotX - rx) / denominator;
    double rearRightPower = (y + rotX - rx) / denominator;

    // Send power to motors
    rearLeftMotor.set(rearLeftPower * SpeedMultiplier);
    rearRightMotor.set(rearRightPower * SpeedMultiplier);
    frontLeftMotor.set(frontLeftPower * SpeedMultiplier);
    frontRightMotor.set(frontRightPower * SpeedMultiplier);
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
