package frc.robot;


import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {
  // Xbox Controller
  private final XboxController m_controller = new XboxController(0);

  // Motors
  private final Victor rearLeftMotor = new Victor(0);
  private final Victor rearRightMotor = new Victor(1);
  private final Victor frontLeftMotor = new Victor(2);
  private final Victor frontRightMotor = new Victor(3);

  // Group of 4 motors
  private final MotorControllerGroup motorGroup = new MotorControllerGroup(rearLeftMotor, rearRightMotor, frontLeftMotor, frontRightMotor);

  // Timer
  private final Timer m_timer = new Timer();

  // State variables
  private boolean motorsOn = true;

  @Override
  public void robotInit() {
    // Invert neccessary motors
    rearRightMotor.setInverted(true);
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
  }


  @Override
  public void teleopPeriodic() {
    // Turn motors on/off - X Button
    if(m_controller.getXButton()) {
      if (motorsOn)
      {
        motorGroup.stopMotor();
        motorsOn = false;
      } else {
        motorGroup.set(1);
        motorsOn = true;
      }
    }

    // Retrive axis from controller
    double y = -m_controller.getLeftY();
    double x = m_controller.getLeftX() * 1.1; // Counteract imperfect strafing
    double rx = m_controller.getRightX();

    // Calculate denominator
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

    // Calculate motor power
    double rearLeftPower = (y - x + rx) / denominator;
    double frontLeftPower = (y + x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double rearRightPower = (y + x - rx) / denominator;

    // Send power to motors
    rearLeftMotor.set(rearLeftPower);
    rearRightMotor.set(rearRightPower);
    frontLeftMotor.set(frontLeftPower);
    frontRightMotor.set(frontRightPower);
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
