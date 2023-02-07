package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;

public final class Controller {
    private final static XboxController mController = new XboxController(RobotMap.XBOX_CONTROLLER);

    public final static double getLeftX() {
        return mController.getLeftX();
    }

    public final static double getLeftY() {
        return mController.getLeftY();
    }

    public final static double getRightX() {
        return mController.getRightX();
    }

    public final static double getRightY() {
        return mController.getRightY();
    }
}
