package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmSubsystemConstants.*;

/**
 * Responsible for moving intake system inwards/outwards.
 * Act as horizontal lift.
 */
public class Subsys_Arm extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, CANSparkMax.MotorType.kBrushless);;
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    public Subsys_Arm() {
        m_motor.restoreFactoryDefaults();

        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(kSmartCurrentLimit);

        m_encoder.setPositionConversionFactor(kDistancePerRevolution);
        m_encoder.setVelocityConversionFactor(kVelocityMetersPerSecond);

        m_pidController.setFeedbackDevice(m_encoder);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIZone);
        m_pidController.setOutputRange(kMinExtend, kMaxExtend);

        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxExtend);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinExtend);

        resetPosition();  // Assumes that it starts at the LOWEST position
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        clampedSpeed = MathUtil.applyDeadband(clampedSpeed, kControllerDeadband);
        m_pidController.setReference(clampedSpeed, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
    }

    public void setPosition(double position) {
        double reference = MathUtil.clamp(position, kMinExtend, kMaxExtend);
        m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, 0, kFeedForwardVelocity);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("arm/position", m_encoder.getPosition());
        SmartDashboard.putNumber("arm/velocity", m_encoder.getVelocity());
    }

    public final void resetPosition() {
        m_encoder.setPosition(kMinExtend);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm");
        builder.addDoubleProperty("encoder position", this::getPosition, this::setPosition);
    }
}