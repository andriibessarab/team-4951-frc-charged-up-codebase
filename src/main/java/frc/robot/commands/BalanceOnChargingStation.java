package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceOnChargingStation extends CommandBase {
    private DriveSubsystem m_drive;
    private final PIDController m_xPositionPIDController = new PIDController(
            Constants.BalancePID.kPosKP,
            Constants.BalancePID.kPosKI,
            Constants.BalancePID.kPosKD
    );
    private final PIDController m_yawPIDController = new PIDController(
            Constants.BalancePID.kYawKP,
            Constants.BalancePID.kYawKI,
            Constants.BalancePID.kYawKD
    );

    public BalanceOnChargingStation(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_xPositionPIDController.setSetpoint(0.0);
        m_yawPIDController.setSetpoint(0.0);
    }

    @Override
    public void execute() {
//        double vx = m_xPositionPIDController.calculate(m_drive.getPitch().getRadians());
//        double omega = m_yawPIDController.calculate(m_drive.getYaw().getRadians());
//
//        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, 0, omega);
//
//        m_drive.drive(chassisSpeeds, true, false);
    }

    @Override
    public boolean isFinished() {
        // Gyro never perfect
        return m_xPositionPIDController.atSetpoint(); // && Math.abs(m_drive.getPitchOmega()) <= 0.1;
    }
}
