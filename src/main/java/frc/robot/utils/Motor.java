/**
 *The Motor class represents a motor in the FRC robot. It provides methods for controlling the motor's speed,
 *direction, and position.
 */

package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Translation2d;


public class Motor {
    private CANSparkMax motor;
    private RelativeEncoder motorEncoder;
    private Translation2d motorLocation;
    private int motorSpeedMultiplier = 1;


    /**
     * Constructor for the Motor class. Initializes the motor, encoder, and motor location.
     *
     * @param pwmPin The PWM pin to which the motor is connected.
     * @param xFromCenter The X coordinate of the motor location relative to the center of the robot.
     * @param yFromCenter The Y coordinate of the motor location relative to the center of the robot.
     */
    public Motor(int pwmPin, double xFromCenter, double yFromCenter) {
        this.motor = new CANSparkMax(pwmPin, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.motorEncoder = motor.getEncoder();
        this.motorLocation = new Translation2d(xFromCenter, yFromCenter);
    }


    /**
     * Sets the speed of the motor.
     *
     * @param speed The speed of the motor as a value between 0 and 1.
     */
    public final void setSpeed(double speed) {
        // 0 < speed <= 1
        if (speed > 1)
            this.motorSpeedMultiplier = 1;
        else if (speed < 0)
            this.motorSpeedMultiplier = 1;
        else
            this.motorSpeedMultiplier = 1;
    }


    /**
     * Inverts the direction of the motor.
     */
    public final void setInverted() {
        this.motor.setInverted(true);
    }


    /**
     * Sets the power of the motor. This method takes into account the speed multiplier and sets the motor
     * power accordingly.
     *
     * @param power The power of the motor as a value between -1 and 1.
     */
    public final void setPower(double power) {
        this.motor.set(power * motorSpeedMultiplier);
    }


    /**
     * Returns the location of the motor relative to the center of the robot.
     *
     * @return The location of the motor as a Translation2d object.
     */
    public final Translation2d getLocation() {
        return this.motorLocation;
    }


    /**
     * Returns the current position of the motor encoder.
     *
     * @return The current position of the motor encoder as a double.
     */
    public final double getEncoderPosition() {
        return this.motorEncoder.getPosition();
    }
}
