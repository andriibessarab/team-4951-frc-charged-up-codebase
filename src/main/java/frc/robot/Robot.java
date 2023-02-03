package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {

  // Xbox Controller
  private final XboxController m_controller = new XboxController(0);

  // Gyro
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry limelightTv = table.getEntry("tv");
  NetworkTableEntry limelightTx = table.getEntry("tx"); 
  NetworkTableEntry limelightTy = table.getEntry("ty");
  NetworkTableEntry limelightTa = table.getEntry("ta");

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
  public void robotPeriodic() {
    // Read limelight values periodically
    boolean limelightHasTarget = limelightTv.getBoolean(false);     // Whether the limelight has any valid targets (0 or 1)
    double limelightHorizontalOffset = limelightTx.getDouble(0.0);  // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    double limelightVerticalOffset = limelightTy.getDouble(0.0);    // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    double limelightTargetArea = limelightTa.getDouble(0.0);        // Target Area (0% of image to 100% of image)  

    // DEBUG - Update smart dashboard periodically
    SmartDashboard.putBoolean("Limelight Has Target", limelightHasTarget);
    SmartDashboard.putNumber("Limelight Horizontal Offset", limelightHorizontalOffset);
    SmartDashboard.putNumber("Limelight Vertical Offset", limelightVerticalOffset);
    SmartDashboard.putNumber("Limelight Target Area", limelightTargetArea);
  }


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
    double rearLeftPower = (rotY - rotX + rx) / denominator;
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double rearRightPower = (rotY + rotX - rx) / denominator;

    // Send power to motors
    rearLeftMotor.set(rearLeftPower * SpeedMultiplier);
    rearRightMotor.set(rearRightPower * SpeedMultiplier);
    frontLeftMotor.set(frontLeftPower * SpeedMultiplier);
    frontRightMotor.set(frontRightPower * SpeedMultiplier);

    // DEBUG - Update smart dashboard periodically
    SmartDashboard.putNumber("Rear Left Motor Power", rearLeftPower);
    SmartDashboard.putNumber("Rear Right Motor Power", rearRightPower);
    SmartDashboard.putNumber("Front Left Motor Power", frontLeftPower);
    SmartDashboard.putNumber("Front Right Motor Power", frontRightPower);
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
