package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain {
    // Motors
    private static final Victor rearLeftMotor = new Victor(0);
    private static final Victor rearRightMotor = new Victor(1);
    private static final Victor frontLeftMotor = new Victor(2);
    private static final Victor frontRightMotor = new Victor(3);


    // Variables
    private static double speedMultiplier = 1;


    // Initialize motors(i.o. invert neccessary motors, etc.)
    public static void initializeMotors() {
        // Invert neccessary motors
        rearRightMotor.setInverted(true);
    } 


    // Set speed multiplier for motors
    public static void setSpeed(double speed) {
        // Check that 0 < speed <= 1
        if (speed > 1)
            speedMultiplier = 1;
        else if (speed < 0)
            speedMultiplier = 0;
        else
            speedMultiplier = speed;
    }


    // Move motors
    public static void drive(double xSpeed,double ySpeed, double zRot, double gyroAngle) {
        ySpeed = ySpeed * 1.1; // Counteract imperfect strafing

        // Calculate denominator
        double botDir = (Math.PI / 180) * gyroAngle;

        // Rotate the movement direction counter to the bot's rotation
        double xRot = xSpeed * Math.cos(-botDir) - ySpeed * Math.sin(-botDir);
        double yRot = xSpeed * Math.sin(-botDir) + ySpeed * Math.cos(-botDir);

        // Calculate deniminator
        double denominator = Math.max(Math.abs(yRot) + Math.abs(xRot) + Math.abs(zRot), 1);

        // Calculate motors power
        double rearLeftPower = (yRot - xRot + zRot) / denominator;
        double frontLeftPower = (yRot + xRot + zRot) / denominator;
        double frontRightPower = (yRot - xRot - zRot) / denominator;
        double rearRightPower = (yRot + xRot - zRot) / denominator;

        // Send power to motors
        rearLeftMotor.set(rearLeftPower * speedMultiplier);
        rearRightMotor.set(rearRightPower * speedMultiplier);
        frontLeftMotor.set(frontLeftPower * speedMultiplier);
        frontRightMotor.set(frontRightPower * speedMultiplier);

        // DEBUG - Update smart dashboard periodically
        SmartDashboard.putNumber("Gyro Angle", gyroAngle);
        SmartDashboard.putNumber("Rear Left Motor Power", rearLeftPower);
        SmartDashboard.putNumber("Rear Right Motor Power", rearRightPower);
        SmartDashboard.putNumber("Front Left Motor Power", frontLeftPower);
        SmartDashboard.putNumber("Front Right Motor Power", frontRightPower);
    }
}

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
