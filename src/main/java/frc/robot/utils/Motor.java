package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

/**
 * The Motor class represents a motor in the FRC robot. It provides methods for
 * controlling the motor's speed,
 * direction, and position.
 */
public class Motor {
    private CANSparkMax motor;
    private RelativeEncoder motorEncoder;
    private int motorSpeedMultiplier = 1;

    /**
     * Constructor for the Motor class. Initializes the motor, encoder, and motor
     * location.
     *
     * @param pwmPin      The PWM pin to which the motor is connected.
     * @param xFromCenter The X coordinate of the motor location relative to the
     *                    center of the robot.
     * @param yFromCenter The Y coordinate of the motor location relative to the
     *                    center of the robot.
     */
    public Motor(int pwmPin) {
        this.motor = new CANSparkMax(pwmPin, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.motorEncoder = motor.getEncoder();
    }

    public final void restoreMotorToFactoryDefaults() {
        this.motor.restoreFactoryDefaults();
    }

    public final void resetEncoder() {
        this.motorEncoder.setPosition(0);
    }

    /**
     * Sets the speed of the motor.
     *
     * @param speed The speed of the motor as a value between 0 and 1.
     */
    public final void setSpeedMultiplier(double speed) {
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
     * Sets the power of the motor. This method takes into account the speed
     * multiplier and sets the motor
     * power accordingly.
     *
     * @param power The power of the motor as a value between -1 and 1.
     */
    public final void setPower(double power) {
        this.motor.set(power * motorSpeedMultiplier);
    }

    public final void setEncoderPosition(double p) {
        this.motorEncoder.setPosition(p);
    }

    public final void setEncoderPositionConversionFactor(double f) {
        this.motorEncoder.setPositionConversionFactor(f);
    }

    public final void setEncoderVelocityConversionFactor(double f) {
        this.motorEncoder.setVelocityConversionFactor(f);
    }

    /**
     * Returns the current position of the motor encoder.
     *
     * @return The current position of the motor encoder as a double.
     */
    public final double getEncoderPosition() {
        return this.motorEncoder.getPosition();
    }

    public final double getEncoderVelocity() {
        return this.motorEncoder.getVelocity();
    }

    public final CANSparkMax getMotorInstance() {
        return this.motor;
    }

    public final RelativeEncoder getEncoderInstance() {
        return this.motorEncoder;
    }
}
