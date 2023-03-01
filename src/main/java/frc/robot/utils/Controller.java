package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;


/**
 * The Controller class provides methods to retrieve values from an XboxController object.
 */
public class Controller {
    /** The threshold value of the controller movement below which the movement is ignored. */
	public static final double CONTROLLER_THRESHOLD = 0.2;

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
     * Returns the current state of the A button on the game controller.
     * @return True if the A button is currently being pressed, false otherwise.
     */
    public final boolean getAButtonState() {
        return controller.getAButton();
    }


    /**
     * Returns true if the A button on the game controller was just pressed during the current cycle.
     * @return True if the A button was just pressed, false otherwise.
     */
    public final boolean getAButtonPressed() {
        return controller.getAButtonPressed();
    }


    /**
     * Returns true if the A button on the game controller was just released during the current cycle.
     * return True if the A button was just released, false otherwise.
     */
    public final boolean getAButtonReleased() {
        return controller.getAButtonReleased();
    }


    /**
     * Returns the current state of the B button on the game controller.
     * @return True if the B button is currently being pressed, false otherwise.
     */
    public final boolean getBButtonState() {
        return controller.getBButton();
    }


    /**
     * Returns true if the B button on the game controller was just pressed during the current cycle.
     * @return True if the B button was just pressed, false otherwise.
     */
    public final boolean getBButtonPressed() {
        return controller.getBButtonPressed();
    }


    /**
     * Returns true if the B button on the game controller was just released during the current cycle.
     * return True if the B button was just released, false otherwise.
     */
    public final boolean getBButtonReleased() {
        return controller.getBButtonReleased();
    }


    /**
     * Returns the current state of the X button on the game controller.
     * @return True if the X button is currently being pressed, false otherwise.
     */
    public final boolean getXButtonState() {
        return controller.getXButton();
    }


    /**
     * Returns true if the X button on the game controller was just pressed during the current cycle.
     * @return True if the X button was just pressed, false otherwise.
     */
    public final boolean getXButtonPressed() {
        return controller.getXButtonPressed();
    }


    /**
     * Returns true if the X button on the game controller was just released during the current cycle.
     * return True if the X button was just released, false otherwise.
     */
    public final boolean getXButtonReleased() {
        return controller.getXButtonReleased();
    }


    /**
     * Returns the current state of the Y button on the game controller.
     * @return True if the Y button is currently being pressed, false otherwise.
     */
    public final boolean getYButtonState() {
        return controller.getYButton();
    }


    /**
     * Returns true if the Y button on the game controller was just pressed during the current cycle.
     * @return True if the Y button was just pressed, false otherwise.
     */
    public final boolean getYButtonPressed() {
        return controller.getYButtonPressed();
    }


    /**
     * Returns true if the Y button on the game controller was just released during the current cycle.
     * return True if the Y button was just released, false otherwise.
     */
    public final boolean getYButtonReleased() {
        return controller.getYButtonReleased();
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
