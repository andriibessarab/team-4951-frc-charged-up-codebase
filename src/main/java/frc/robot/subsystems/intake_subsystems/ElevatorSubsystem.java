package frc.robot.subsystems.intake_subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorSubsystemConstants.*;

/**
 * Responsible for raising/lowering the elevator.
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final CANSparkMax m_motor = new CANSparkMax(Constants.ElevatorSubsystemConstants.kMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);;
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_pidController = m_motor.getPIDController();

    private final int MOVE_UP_PID_SLOT = 0;
    private final int MOVE_DOWN_PID_SLOT = 1;

    public ElevatorSubsystem() {
        m_motor.restoreFactoryDefaults();

        m_motor.setIdleMode(Constants.ElevatorSubsystemConstants.kMotorMode);
        m_motor.setInverted(Constants.ElevatorSubsystemConstants.kMotorInverted);
        m_motor.setSmartCurrentLimit(Constants.ElevatorSubsystemConstants.kSmartCurrentLimit);

        m_encoder.setPositionConversionFactor(Constants.ElevatorSubsystemConstants.kDistancePerRevolution);
        m_encoder.setVelocityConversionFactor(Constants.ElevatorSubsystemConstants.kVelocityMetersPerSecond);

        m_pidController.setFeedbackDevice(m_encoder);

        // Setup PID Slot for moving upwards
        m_pidController.setP(Constants.ElevatorSubsystemConstants.MOVE_UP.kP, MOVE_UP_PID_SLOT);
        m_pidController.setI(Constants.ElevatorSubsystemConstants.MOVE_UP.kI, MOVE_UP_PID_SLOT);
        m_pidController.setD(Constants.ElevatorSubsystemConstants.MOVE_UP.kD, MOVE_UP_PID_SLOT);
        m_pidController.setIZone(Constants.ElevatorSubsystemConstants.MOVE_UP.kIZone, MOVE_UP_PID_SLOT);
        m_pidController.setOutputRange(kMinHeight, kMaxHeight, MOVE_UP_PID_SLOT);

        // Setup PID Slot for moving downwards
        m_pidController.setP(Constants.ElevatorSubsystemConstants.MOVE_DOWN.kP, MOVE_DOWN_PID_SLOT);
        m_pidController.setI(Constants.ElevatorSubsystemConstants.MOVE_DOWN.kI, MOVE_DOWN_PID_SLOT);
        m_pidController.setD(Constants.ElevatorSubsystemConstants.MOVE_DOWN.kD, MOVE_DOWN_PID_SLOT);
        m_pidController.setIZone(Constants.ElevatorSubsystemConstants.MOVE_DOWN.kIZone, MOVE_DOWN_PID_SLOT);
        m_pidController.setOutputRange(kMinHeight, kMaxHeight, MOVE_DOWN_PID_SLOT);

        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kMaxHeight);  // Top distance limit
        m_motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)kMinHeight);  // Bottom distance limit

        resetPosition();  // Assumes that it starts at the LOWEST position

        SmartDashboard.putNumber("Elevator/PosFactor", Constants.ElevatorSubsystemConstants.kDistancePerRevolution);
        SmartDashboard.putNumber("Elevator/VelFactor", Constants.ElevatorSubsystemConstants.kVelocityMetersPerSecond);
        SmartDashboard.putNumber("Elevator/MaxHeight", kMaxHeight);
        SmartDashboard.putNumber("Elevator/MinHeight", kMinHeight);
    }

    public void setSpeed(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, kMaxControllerDownSpeed, kMaxControllerUpSpeed);
        clampedSpeed = MathUtil.applyDeadband(clampedSpeed, kControllerDeadband);
        m_pidController.setReference(clampedSpeed, CANSparkMax.ControlType.kDutyCycle, MOVE_UP_PID_SLOT, MOVE_UP.kFeedForwardVelocity);
        updateSmartDashboard();
    }

    public void setSpeed1(double speed){
        m_motor.set(speed);
    }

    public void setPosition(double position) {
        double reference = MathUtil.clamp(position, kMinHeight, kMaxHeight);
        double current = m_encoder.getPosition();
        if (reference < current) {
            // We want to go down... 
            m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, MOVE_DOWN_PID_SLOT,
                    MOVE_DOWN.kFeedForwardVelocity);
        } else {
            
            m_pidController.setReference(reference, CANSparkMax.ControlType.kPosition, MOVE_UP_PID_SLOT,
                    MOVE_UP.kFeedForwardVelocity);
        }
        updateSmartDashboard();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void stop() {
        m_pidController.setReference(0, CANSparkMax.ControlType.kDutyCycle, MOVE_UP_PID_SLOT, MOVE_UP.kFeedForwardVelocity);
    }

    public void stop1(){
        m_motor.stopMotor();
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