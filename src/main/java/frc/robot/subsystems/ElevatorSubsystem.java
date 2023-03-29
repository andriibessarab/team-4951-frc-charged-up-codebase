package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorSubsystem.*;

/**
 * Responsible for raising/lowering the elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(Constants.ElevatorSubsystem.kMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    public ElevatorSubsystem() {
        m_motor.restoreFactoryDefaults();

        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(Constants.ElevatorSubsystem.kSmartCurrentLimit);

        m_encoder.setPositionConversionFactor(Constants.ElevatorSubsystem.kDistancePerRevolution);
        m_encoder.setVelocityConversionFactor(Constants.ElevatorSubsystem.kVelocityMetersPerSecond);

        m_pidController.setFeedbackDevice(m_encoder);

        m_pidController.setP(Constants.ElevatorSubsystem.kP);
        m_pidController.setI(Constants.ElevatorSubsystem.kI);
        m_pidController.setD(Constants.ElevatorSubsystem.kD);
        m_pidController.setIZone(Constants.ElevatorSubsystem.kIZone);
        m_pidController.setOutputRange(kMinHeight, kMaxHeight);

        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxHeight);  // Top distance limit
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinHeight);  // Bottom distance limit

        resetPosition();  // Assumes that it starts at the LOWEST position

        SmartDashboard.putNumber("Elevator/PosFactor", Constants.ElevatorSubsystem.kDistancePerRevolution);
        SmartDashboard.putNumber("Elevator/VelFactor", Constants.ElevatorSubsystem.kVelocityMetersPerSecond);
        SmartDashboard.putNumber("Elevator/MaxHeight", kMaxHeight);
        SmartDashboard.putNumber("Elevator/MinHeight", kMinHeight);
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        clampedSpeed = MathUtil.applyDeadband(clampedSpeed, kControllerDeadband);
        m_pidController.setReference(clampedSpeed, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
        updateSmartDashboard();
    }

    public void setPosition(double position) {
        double reference = MathUtil.clamp(position, kMinHeight, kMaxHeight);
        double current = m_encoder.getPosition();
        if (reference < current) {
            // We want to go down... but feed forward prevents unless negative. Actually all the PID
            // values would be in reverse but not sure what the best approach is - toggling PID values
            // every time direction changes does not feel right... but presently it is slow to drop.
            m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, 0, -kFeedForwardVelocity);
        } else {
            m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, 0, kFeedForwardVelocity);
        }
        updateSmartDashboard();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle, 0, kFeedForwardVelocity);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Elevator/Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Elevator/Velocity", m_encoder.getVelocity());
    }

    public final void resetPosition() {
        m_encoder.setPosition(kMinHeight);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("encoder position", this::getPosition, this::setPosition);
    }
}
