package frc.robot.controller.devices;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controller.Controller;
import edu.wpi.first.wpilibj.XboxController;

public class XBoxControllerDevice extends Controller {
    edu.wpi.first.wpilibj.XboxController controller;

    public XBoxControllerDevice(int port) {
        controller = new edu.wpi.first.wpilibj.XboxController(port);
    }

    @Override
    public GenericHID getGenericHID() {
        return controller;
    }

    @Override
    protected double getRawLeftX() {
        return controller.getLeftX();
    }

    @Override
    protected double getRawLeftY() {
        return controller.getLeftY();
    }

    @Override
    protected double getRawRightX() {
        return controller.getRightX();
    }

    @Override
    protected double getRawRightY() {
        return controller.getRightY();
    }

    protected double getRawRightTriggerAxis() { return controller.getRightTriggerAxis(); }

    protected double getRawLeftTriggerAxis() { return controller.getLeftTriggerAxis(); }
}
