package frc.robot.controller;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.controller.devices.XBoxControllerDevice;

public abstract class Controller {
    double m_xLeftOffset = 0.0, m_yLeftOffset = 0.0, m_xRightOffset = 0.0, m_yRightOffset = 0.0;
    double m_xLeftDeadband = 0.0, m_yLeftDeadband = 0.0, m_xRightDeadband = 0.0, m_yRightDeadband = 0.0;

    public static Controller create(int port) {
        Controller controller = new XBoxControllerDevice(port);
        controller.calibrate();
        return controller;
    }

    public abstract GenericHID getGenericHID();

    public void setDeadbandLeftX(double deadband) {
        m_xLeftDeadband = deadband;
    }

    public void setDeadbandLeftY(double deadband) {
        m_yLeftDeadband = deadband;
    }

    public void setDeadbandRightX(double deadband) {
        m_xRightDeadband = deadband;
    }

    public void setDeadbandRightY(double deadband) {
        m_yRightDeadband = deadband;
    }

    /**
     * Calibrate assumes the controller is at rest and finds the offsets from zero.
     */
    public void calibrate() {
        m_xLeftOffset = getRawLeftX();
        m_yLeftOffset = getRawLeftY();
        m_xRightOffset = getRawRightX();
        m_yRightOffset = getRawRightY();
    }

    public double getLeftX() {
        return MathUtil.applyDeadband(getRawLeftX() - m_xLeftOffset, m_xLeftDeadband);
    }

    public double getLeftY() {
        return MathUtil.applyDeadband(getRawLeftY() - m_yLeftOffset, m_yLeftDeadband);
    }

    public double getRightX() {
        return MathUtil.applyDeadband(getRawRightX() - m_xRightOffset, m_xRightDeadband);
    }

    public double getRightY() {
        return MathUtil.applyDeadband(getRawRightY() - m_yRightOffset, m_yRightDeadband);
    }

    public double getLeftTriggerAxis() {
        return getRawLeftTriggerAxis();
    }

    public double getRightTriggerAxis() {
        return getRawRightTriggerAxis();
    }

    protected abstract double getRawLeftX();

    protected abstract double getRawLeftY();

    protected abstract double getRawRightX();

    protected abstract double getRawRightY();

    protected abstract double getRawLeftTriggerAxis();

    protected abstract double getRawRightTriggerAxis();

}
