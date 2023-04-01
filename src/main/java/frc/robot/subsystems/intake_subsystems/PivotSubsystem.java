package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.PivotSubsystem.*;

/**
 * Responsible for moving claw inwards/outwards.
 */
public class PivotSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(kMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);;
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    @Override
    public void periodic() {
        // TODO Auto-generated method stub

    }

    public PivotSubsystem() {
        m_motor.restoreFactoryDefaults();

        m_motor.setInverted(false);

        // m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake); // #TODO test
        m_motor.setSmartCurrentLimit(kSmartCurrentLimit);

        m_encoder.setPositionConversionFactor(kDistancePerRevolution);
        m_encoder.setVelocityConversionFactor(kVelocityMetersPerSecond);

        m_pidController.setFeedbackDevice(m_encoder);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIZone);
        m_pidController.setOutputRange(kMinOut, kMaxOut);

        //m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        //m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        //m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxOut);  // Top distance limit
        //m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinOut);  // Bottom distance limit

        resetPosition();  // Assumes that it starts at the LOWEST position

        SmartDashboard.putNumber("Pivot/PosFactor", Constants.PivotSubsystem.kDistancePerRevolution);
        SmartDashboard.putNumber("Pivot/VelFactor", Constants.PivotSubsystem.kVelocityMetersPerSecond);
        SmartDashboard.putNumber("Pivot/MaxAngle", kMaxOut);
        SmartDashboard.putNumber("Pivot/MinAngle", kMinOut);
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        //clampedSpeed = MathUtil.applyDeadband(clampedSpeed, kControllerDeadband);
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
        m_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
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
