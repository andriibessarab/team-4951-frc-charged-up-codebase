package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;

public final class Controller {

    private final static XboxController mController = new XboxController(RobotMap.XBOX_CONTROLLER);


    // Left stick - X Value
    public final static double getLeftX(boolean thresholded) {
        return thresholded ? getThresholdedValue(mController.getLeftX()) : mController.getLeftX();
    }

    // Left stick - Y Value
    public final static double getLeftY(boolean thresholded) {
        return thresholded ? -getThresholdedValue(mController.getLeftY()) : -mController.getLeftY();
    }

    // Right stick - X Value
    public final static double getRightX(boolean thresholded) {
        return thresholded ? getThresholdedValue(mController.getRightX()) : mController.getRightX();
    }

    // Right Stick - Y Value
    public final static double getRightY(boolean thresholded) {
        return thresholded ? getThresholdedValue(mController.getRightY()) : mController.getRightY();
    }

    /* THRESHOLD
     * Threshold is the value below which the parameter will be automatically set to zero.
     * You can change this value in frc.robot.RobotMap class
     */
    private static final double getThresholdedValue(double v) {
        return Math.abs(v) > RobotMap.CONTROLLER_THRESHOLD ? v : 0;
    }
}
