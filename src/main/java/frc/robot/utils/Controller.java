/**
 * The Controller class provides methods to retrieve values from an XboxController object and apply a controller threshold.
 */

package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.RobotMap;


public class Controller {
    private XboxController controller;


    /**
     * Constructs a new Controller object with the specified PWM pin number.
     *
     * @param pwmPin the PWM pin number that the controller is connected to
     */
    public Controller(int pwmPin) {
        this.controller = new XboxController(pwmPin);
    }


    /**
     * Returns the raw value of the left stick X-axis of the controller.
     *
     * @return the raw value of the left stick X-axis
     */
    public final double getLeftX() {
        return controller.getLeftX();
    }


    /**
    * Returns the thresholded value of the left stick X-axis of the controller.
    *
    * @return the thresholded value of the left stick X-axis
    */
    public final double getThresholdedLeftX() {
        return getThresholdedValue(controller.getLeftX());
    }


    /**
     * Returns the raw value of the left stick Y-axis of the controller.
     *
     * @return the raw value of the left stick Y-axis
     */
    public final double getLeftY() {
        return -controller.getLeftY();
    }


    /**
    * Returns the thresholded value of the left stick Y-axis of the controller.
    *
    * @return the thresholded value of the left stick Y-axis
    */
    public final double getThresholdedLeftY() {
        return getThresholdedValue(-controller.getLeftY());
    }

        /**
     * Returns the raw value of the right stick X-axis of the controller.
     *
     * @return the raw value of the right stick X-axis
     */
    public final double getRightX() {
        return controller.getRightX();
    }


    /**
    * Returns the thresholded value of the right stick X-axis of the controller.
    *
    * @return the thresholded value of the right stick X-axis
    */
    public final double getThresholdedRightX() {
        return getThresholdedValue(controller.getRightX());
    }


    /**
     * Returns the raw value of the right stick Y-axis of the controller.
     *
     * @return the raw value of the right stick Y-axis
     */
    public final double getRightY() {
        return -controller.getRightY();
    }


    /**
    * Returns the thresholded value of the right stick Y-axis of the controller.
    *
    * @return the thresholded value of the right stick Y-axis
    */
    public final double getThresholdedRightY() {
        return getThresholdedValue(-controller.getRightY());
    }


    /**
     * Applies a controller threshold to the input value.
     *
     * @param v the input value
     * @return the thresholded or raw input value
     */
    private static final double getThresholdedValue(double v) {
        return Math.abs(v) > RobotMap.CONTROLLER_THRESHOLD ? v : 0;
    }
}
