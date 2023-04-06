package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PivotSubsystemConstants.*;

/**
 * Responsible for moving claw inwards/outwards.
 */
public class PivotSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);;
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    public PivotSubsystem() {
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(kMotorInverted);

        m_motor.setIdleMode(kMotorMode);
        m_motor.setSmartCurrentLimit(kSmartCurrentLimit);

        m_encoder.setPositionConversionFactor(kDistancePerRevolution);
        m_encoder.setVelocityConversionFactor(kVelocityMetersPerSecond);

        m_pidController.setFeedbackDevice(m_encoder);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIZone);
        m_pidController.setOutputRange(-0.5, 0.5);

        // #TODO soft limit was disabled will it still work?
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxOut);  // Top distance limit
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinOut);  // Bottom distance limit

        resetPosition();  // Assumes that it starts at the LOWEST position
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        clampedSpeed = MathUtil.applyDeadband(clampedSpeed, kControllerDeadband);  // #TODO soft limit was disabled will it still work?
        m_pidController.setReference(clampedSpeed, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
        updateSmartDashboard();
    }

    public void setPosition(double position) {
        double reference = MathUtil.clamp(position, kMinOut, kMaxOut);
        m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, 0, kFeedForwardVelocity);
        updateSmartDashboard();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        
        m_motor.set(0);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Pivot/Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Pivot/Velocity", m_encoder.getVelocity());
        //TODO: check if .setsoftlimit resets everytime it is called
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) SmartDashboard.getNumber("Pivot/MaxAngle", kMaxOut));
        SmartDashboard.putNumber("close time", 0.5);
        SmartDashboard.putNumber("close speed", -0.2);
        SmartDashboard.putNumber("open time", 0.5);
        SmartDashboard.putNumber("open speed", 0.2);
    }

    public final void resetPosition() {
        m_encoder.setPosition(kMinOut);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        builder.addDoubleProperty("encoder position", this::getPosition, this::setPosition);
    }
}
