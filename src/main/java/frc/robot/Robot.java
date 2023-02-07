package frc.robot;


import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends TimedRobot {
  // This will fix all your problems!!!
  public static final String Jason = "Jason wrote this line of code";
  
  // Xbox Controller
  private final XboxController mController = new XboxController(0);
  
  // Gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  // Limelight
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry limelightTv = table.getEntry("tv");
  private final NetworkTableEntry limelightTx = table.getEntry("tx"); 
  private final NetworkTableEntry limelightTy = table.getEntry("ty");
  private final NetworkTableEntry limelightTa = table.getEntry("ta");

  // Motors
  private final Victor rearLeftMotor = new Victor(0);
  private final Victor rearRightMotor = new Victor(1);
  private final Victor frontLeftMotor = new Victor(2);
  private final Victor frontRightMotor = new Victor(3);

  // PathWeaver
  // private final String trajectoryJSON = "paths/Test.wpilib.json";
  // pruvate Trajectory trajectory = new Trajectory();

  // Timer
  private final Timer mTimer = new Timer();

  // Variables
  private double SpeedMultiplier = 0.5; // 0 < motorSpeed <= 1 Change this to reduce overall speed of motorls
  //private double angle = 0; 


  @Override
  public void robotInit() {
    // Invert neccessary motors
    rearRightMotor.setInverted(true);

    // Calibrate gyroscope
    gyro.calibrate();

    // Pathweaver retrieve path(s)
    /*try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }*/
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
    mTimer.reset();
    mTimer.start();
  }


  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {}


  @Override
  public void teleopPeriodic() {
    // Retrive axis from controller
    double y = mController.getLeftY();
    double x = mController.getLeftX() * 1.1; // Counteract imperfect strafing
    double rx = mController.getRightX();

    // Calculate denominator
    double denominator;

    double rearLeftPower=0;
    double frontLeftPower=0;
    double rearRightPower=0;
    double frontRightPower=0;

    double botHeading = (Math.PI / 180) * gyro.getAngle();

    // Rotate the movement direction counter to the bot's rotation
    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
    denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    rearLeftPower = (rotY - rotX + rx) / denominator;
    frontLeftPower = (rotY + rotX + rx) / denominator;
    frontRightPower = (rotY - rotX - rx) / denominator;
    rearRightPower = (rotY + rotX - rx) / denominator;


    /*if(rx<0.2&&rx>0.2){
      // Calculate motor power

      double botHeading = (Math.PI / 180) * gyro.getAngle();

      // Rotate the movement direction counter to the bot's rotation
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
      denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
      rearLeftPower = (rotY - rotX + rx) / denominator;
      frontLeftPower = (rotY + rotX + rx) / denominator;
      frontRightPower = (rotY - rotX - rx) / denominator;
      rearRightPower = (rotY + rotX - rx) / denominator;
    } else {
      // Calculate motor power
      denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
      rearLeftPower = (y - x + rx) / denominator;
      frontLeftPower = (y + x + rx) / denominator;
      frontRightPower = (y - x - rx) / denominator;
      rearRightPower = (y + x - rx) / denominator;
      gyro.reset();
    }*/



    // Send power to motors
    rearLeftMotor.set(rearLeftPower * SpeedMultiplier);
    rearRightMotor.set(rearRightPower * SpeedMultiplier);
    frontLeftMotor.set(frontLeftPower * SpeedMultiplier);
    frontRightMotor.set(frontRightPower * SpeedMultiplier);

    // DEBUG - Update smart dashboard periodically
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
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
