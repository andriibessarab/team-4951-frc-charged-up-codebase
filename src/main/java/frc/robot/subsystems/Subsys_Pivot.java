package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PivotSubsystem.*;

/**
 * Responsible for moving claw inwards/outwards with 180 degree rotation.
 */
public class Subsys_Pivot extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, CANSparkMax.MotorType.kBrushless);;
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    @Override
    public void periodic() {
        updateSmartDashboard();

    }

    public Subsys_Pivot() {
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(false);

        // m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(kSmartCurrentLimit);

        m_encoder.setPositionConversionFactor(kDistancePerRevolution);
        m_encoder.setVelocityConversionFactor(kVelocityMetersPerSecond);

        m_pidController.setFeedbackDevice(m_encoder);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIZone);
        m_pidController.setOutputRange(-1, 1);

        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxOut);
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinOut);

        resetPosition();  // Assumes that it starts at the LOWEST position
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        m_motor.set(clampedSpeed);
    }

    public void setPosition(double position) {
        double reference = MathUtil.clamp(position, kMinOut, kMaxOut);
        m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, 0, kFeedForwardVelocity);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_pidController.setReference(getPosition(), CANSparkMax.ControlType.kPosition, 0, kFeedForwardVelocity);

    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("pivot/position", m_encoder.getPosition());
        SmartDashboard.putNumber("pivot/velocity", m_encoder.getVelocity());
    }

    public final void resetPosition() {
        m_encoder.setPosition(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        builder.addDoubleProperty("encoder position", this::getPosition, this::setPosition);
    }
}