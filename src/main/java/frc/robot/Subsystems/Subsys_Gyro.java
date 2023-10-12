
package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;        

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 gyro
 */
public class Subsys_Gyro extends SubsystemBase {
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    public Subsys_Gyro() {}

    /**
     * forward - +ve
     * backward - -ve
     * @return
     */
    public double getPitch() {
        return -m_gyro.getPitch();
    }

    /**
     * right - +ve
     * left - -ve
     * @return
     */
    public double getRoll() {
        return -m_gyro.getRoll();
    }

    /*
     * clockwise pos
     *
     */
    public double getYaw() {
        return m_gyro.getYaw();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
        SmartDashboard.putNumber("Gyro Roll", getRoll());
        SmartDashboard.putNumber("Gyro Yaw", getYaw());
    }

}