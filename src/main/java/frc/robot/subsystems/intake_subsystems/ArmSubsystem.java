package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ArmSubsystem.*;

/**
 * Responsible for moving intake system inwards/ouwards.
 * Act as horizontal lift.
 */
public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);;
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    public ArmSubsystem() {
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

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxExtend);  // Top distance limit
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinExtend);  // Bottom distance limit

        resetPosition();  // Assumes that it starts at the LOWEST position

        SmartDashboard.putNumber("Arm/PosFactor", Constants.ArmSubsystem.kDistancePerRevolution);
        SmartDashboard.putNumber("Arm/VelFactor", Constants.ArmSubsystem.kVelocityMetersPerSecond);
        SmartDashboard.putNumber("Arm/MaxExtend", kMaxExtend);
        SmartDashboard.putNumber("Arm/MinExtend", kMinExtend);
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        clampedSpeed = MathUtil.applyDeadband(clampedSpeed, kControllerDeadband);
        m_pidController.setReference(clampedSpeed, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
        updateSmartDashboard();
    }

    public void setPosition(double position) {
        double reference = MathUtil.clamp(position, kMinExtend, kMaxExtend);
        m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, 0, kFeedForwardVelocity);
        updateSmartDashboard();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Arm/Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Arm/Velocity", m_encoder.getVelocity());
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
