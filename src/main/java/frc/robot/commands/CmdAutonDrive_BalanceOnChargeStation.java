package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Subsys_Drive;

/**
 * Balances the robot on the Charge Station
 */
public class CmdAutonDrive_BalanceOnChargeStation extends CommandBase {
    
    private final Subsys_Drive m_driveTrain;
    double pitchAngleDegrees;
    static double kOffBalanceAngleThresholdDegrees = 0;
    static double kOonBalanceAngleThresholdDegrees = 0;

    public CmdAutonDrive_BalanceOnChargeStation(Subsys_Drive drive) {
        m_driveTrain = drive;
        addRequirements(m_driveTrain);
     }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pitchAngleDegrees = m_driveTrain.getPitch();
        double velo = Math.sin(pitchAngleDegrees * (Math.PI / 180.0)) * 1;

        if(Math.abs(kOffBalanceAngleThresholdDegrees) < pitchAngleDegrees){
            kOffBalanceAngleThresholdDegrees=pitchAngleDegrees;
        } else{
            velo=0;
        }

        // if (xAxisRate > 0) {
        //     xAxisRate = 0.2;
        // } else if (xAxisRate < 0) {
        //     xAxisRate = -0.2;
        // } else {
        //     xAxisRate = 0;
        // }

        m_driveTrain.driveMecanum(0, velo, 0);
    }

    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.driveMecanum(0.0, 0.0,0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
 
        if(Math.abs(m_driveTrain.getRoll())>1){
            return true;
        }

        return false;
    }
}