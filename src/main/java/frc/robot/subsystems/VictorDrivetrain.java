package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.RobotMap;


public final class VictorDrivetrain {

    public  final Victor flM;
    public  final Victor frM;
    public  final Victor rlM;
    public  final Victor rrM;


    public VictorDrivetrain() {
        flM = new Victor(RobotMap.FRONT_LEFT_MOTOR_PWM_PIN);
        frM = new Victor(RobotMap.FRONT_RIGHT_MOTOR_PWM_PIN);
        rlM = new Victor(RobotMap.REAR_LEFT_MOTOR_PWM_PIN);
        rrM = new Victor(RobotMap.REAR_RIGHT_MOTOR_PWM_PIN);

        rrM.setInverted(true);
    }


    // Robot oriented drive(w/ single axis control)
    public final  void driveSingleAxis(double x, double y, double z) {
        if (Math.abs(y) > Math.abs(x) && Math.abs(y) > Math.abs(z)) { // Y-Axis Motion
            setPower(y, y, y, y);
        } else if (Math.abs(x) > Math.abs(y) && Math.abs(x) > Math.abs(z)) { // X-Axis Motion
            if(x > 0) {
                setPower(x*1.1, x*-0.95, x*-1.1, x);
                if(Math.abs(x) >= 0.5) {
                    setPower(x*1.1, x*-0.95, x*-1.1, x);
                } else if(Math.abs(x) > 0.35) {
                    setPower(x*0.9, x*-0.9, x*-1.1, x);

                }
            } else if(x < 0) {
                if(Math.abs(x) >= 0.5) {
                    setPower(x*1.05, x*-1, x*-1, x);
                } else if(Math.abs(x) > 0.35) {
                    setPower(x*1.1, x*-1, x*-1.1, x);
                }
            }
        } else if (Math.abs(z) > Math.abs(y) && Math.abs(z) > Math.abs(x)) { // Z-Axis Movement
            setPower(z, -z, z, -z);
        } else {
            setPower(0, 0, 0, 0);
        }
    }

    // Field oriented drive (w/ gyroscope)
    public final void driveFieldOriented(double xSpeed,double ySpeed, double zRot, double gyroAngle) {
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
        setPower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    // Field oriented drive (multi-axis)
    public final void driveRobotOriented(double xSpeed,double ySpeed, double zRot) {
        ySpeed = ySpeed * 1.1; // Counteract imperfect strafing
    
        // Calculate deniminator
        double denominator = Math.max(Math.abs(ySpeed) + Math.abs(xSpeed) + Math.abs(zRot), 1);
    
        // Calculate motors power
        double rearLeftPower = (ySpeed - xSpeed + zRot) / denominator;
        double frontLeftPower = (ySpeed + xSpeed + zRot) / denominator;
        double frontRightPower = (ySpeed - xSpeed - zRot) / denominator;
        double rearRightPower = (ySpeed + xSpeed - zRot) / denominator;
    
        // Send power to motors
        setPower(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
    }

    private void setPower(double flP, double frP, double rlP, double rrP) {
        flM.set(flP);
        frM.set(frP);
        rlM.set(rlP);
        rrM.set(rrP);
    }
}