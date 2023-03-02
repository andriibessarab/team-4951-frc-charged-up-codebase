package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Controller class provides methods to retrieve values from an
 * XboxController object.
 */
public class Controller extends XboxController {
    /**
     * The threshold value of the controller movement below which the movement is
     * ignored.
     */
    private static final double CONTROLLER_THRESHOLD = 0.2;

    /**
     * Constructs a new Controller object with the specified PWM pin number.
     *
     * @param pwmPin the PWM pin number that the controller is connected to
     */
    public Controller(int pwmPin) {
        super(pwmPin);
    }

    /**
     * Returns the raw value of the left stick X-axis of the controller.
     *
     * @return the raw value of the left stick X-axis
     */
    @Override
    public double getLeftX() {
        return super.getLeftX();
    }

    /**
     * Returns the thresholded value of the left stick X-axis of the controller.
     *
     * @return the thresholded value of the left stick X-axis
     */
    public double getThresholdedLeftX() {
        return getThresholdedValue(getLeftX());
    }

    /**
     * Returns the raw value of the left stick Y-axis of the controller.
     *
     * @return the raw value of the left stick Y-axis
     */
    @Override
    public double getLeftY() {
        return -super.getLeftY();
    }

    /**
     * Returns the thresholded value of the left stick Y-axis of the controller.
     *
     * @return the thresholded value of the left stick Y-axis
     */
    public double getThresholdedLeftY() {
        return getThresholdedValue(getLeftY());
    }

    /**
     * Returns the raw value of the right stick X-axis of the controller.
     *
     * @return the raw value of the right stick X-axis
     */
    @Override
    public double getRightX() {
        return super.getRightX();
    }

    /**
     * Returns the thresholded value of the right stick X-axis of the controller.
     *
     * @return the thresholded value of the right stick X-axis
     */
    public double getThresholdedRightX() {
        return getThresholdedValue(getRightX());
    }

    /**
     * Returns the raw value of the right stick Y-axis of the controller.
     *
     * @return the raw value of the right stick Y-axis
     */
    @Override
    public double getRightY() {
        return -super.getRightY();
    }

    /**
     * Returns the thresholded value of the right stick Y-axis of the controller.
     *
     * @return the thresholded value of the right stick Y-axis
     */
    public double getThresholdedRightY() {
        return getThresholdedValue(getRightY());
    }

    /**
     * Applies a controller threshold to the input value.
     *
     * @param v the input value
     * @return the thresholded input value
     */
    private static final double getThresholdedValue(double v) {
        return Math.abs(v) > CONTROLLER_THRESHOLD ? v : 0;
    }
}
