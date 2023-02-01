package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {
  // Xbox Controller
  private final XboxController m_controller = new XboxController(0);

  // Motors
  private final Victor backLeftMotor = new Victor(0);
  private final Victor backRightMotor = new Victor(1);
  private final Victor frontLeftMotor = new Victor(2);
  private final Victor frontRightMotor = new Victor(3);

  // Motor groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
  
  // Mecanum Drive
  public final MecanumDrive m_drive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  // Timer
  private final Timer m_timer = new Timer();

  @Override
  public void robotInit() {
    // Disable safety for mecanum drive
    m_drive.setSafetyEnabled(false);
    m_drive.feed();
  }


  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
    // Reset timer once autonomous enabled
    m_timer.reset();
    m_timer.start();
  }


  @Override
  public void autonomousPeriodic() {
  }


  @Override
  public void teleopInit() {
    m_drive.setSafetyEnabled(false);
    m_drive.feed();
  }


  @Override
  public void teleopPeriodic() {
    // Drive using xbox controller
    m_drive.driveCartesian(
      m_controller.getLeftX(), m_controller.getLeftY(), m_controller.getPOV()
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
