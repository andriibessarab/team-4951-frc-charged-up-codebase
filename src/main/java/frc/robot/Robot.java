package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {
  // Xbox Controller
  private final XboxController m_controller = new XboxController(0);

  // Motors
  private final Victor frontLeftMotor = new Victor(2);
  private final Victor frontRightMotor = new Victor(3);
  private final Victor rearLeftMotor = new Victor(0);
  private final Victor rearRightMotor = new Victor(1);

  // Mecanum Drive
  public final MecanumDrive m_drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  // Timer
  private final Timer m_timer = new Timer();

  @Override
  public void robotInit() {
    // Disable robot safety
    m_drive.setSafetyEnabled(false);
    m_drive.feed();
    //rearLeftMotor.setInverted(true);
    rearRightMotor.setInverted(true);
    //frontLeftMotor.setInverted(true);

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
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {
    // Disable robot safety
    m_drive.setSafetyEnabled(false);
    m_drive.feed();
  }


  @Override
  public void teleopPeriodic() {
    // Emergency stop driving - X Button
    if(m_controller.getXButton()) {
      m_drive.stopMotor();
    }

    double y = -m_controller.getLeftY();
    double x = m_controller.getLeftX() * 1.1;
    double rx = m_controller.getRightX();

    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    if(frontLeftPower>0.5){
      frontLeftPower=0.5;
    }
    if(backLeftPower>0.5){
      backLeftPower=0.5;
    }
    if(frontRightPower>0.5){
      frontRightPower=0.5;
    }
    if(backRightPower>0.5){
      backRightPower=0.5;
    }

    frontLeftMotor.set(frontLeftPower);
    rearLeftMotor.set(backLeftPower);
    frontRightMotor.set(frontRightPower);
    rearRightMotor.set(backRightPower);
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
