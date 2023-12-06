package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Helpers.LimelightHelpers;
import frc.robot.Subsystems.Subsys_Drive;

public class CmdAlignWithLLTurning extends CommandBase{
    
    private final Subsys_Drive m_driveTrain;
    String limelightName;
    double tx, rx, ta;
    int caseNumber = 1;

    public CmdAlignWithLLTurning(Subsys_Drive drive, String limelight_name) {
        m_driveTrain = drive;
        addRequirements(m_driveTrain);
        limelightName = limelight_name;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        LimelightHelpers.setPipelineIndex(limelightName, 1);
        boolean targetDetected = LimelightHelpers.getTV(limelightName);
        SmartDashboard.putBoolean("detected", targetDetected);

        Pose3d botPose = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);

        tx = LimelightHelpers.getTX(limelightName);
        ta = LimelightHelpers.getTA(limelightName);
        rx = botPose.getRotation().getY()*45;
        SmartDashboard.putNumber("X", LimelightHelpers.getTX(limelightName));
        SmartDashboard.putNumber("A", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putNumber("rX", botPose.getRotation().getY()*45);
        if(targetDetected) {
            switch (caseNumber){
                case 1: //rotate to face april tag, use rx
                    if(rx>10){
                        SmartDashboard.putString("driving", "turning right");
                        m_driveTrain.driveMecanum(0, 0, 0.3);
                    } else if(rx<-10){
                        m_driveTrain.driveMecanum(0, 0, -0.3);
                        SmartDashboard.putString("driving", "turning left");
                    } else{
                        m_driveTrain.driveMecanum(0, 0, 0);
                        caseNumber=2;
                    }
                    break;
                case 2: //strafe to be infront of april tag, use tx
                    if(Math.abs(rx)>10){
                        caseNumber = 1;
                        break;
                    }
                    if(tx>3){
                        SmartDashboard.putString("driving", "right");
                        m_driveTrain.driveMecanum(0.3, 0, 0);
                    } else if(tx<-3){
                        m_driveTrain.driveMecanum(-0.3, 0, 0);
                        SmartDashboard.putString("driving", "left");
                    } else{
                        m_driveTrain.driveMecanum(0, 0, 0);
                        caseNumber=3;
                    }
                    break;
                case 3: //drive to be at right distance, use ta(or something better)
                    SmartDashboard.putString("driving", "forward");
                    if(Math.abs(rx)>10){
                        caseNumber = 1;
                        break;
                    }
                    if(Math.abs(tx)>3){
                        caseNumber = 2;
                        break;
                    }
                    if(ta<1.5){ //play with this value
                        m_driveTrain.driveMecanum(0, 0.3, 0);
                    } else{
                        m_driveTrain.driveMecanum(0, 0, 0);
                        caseNumber=1; //goes back to check everything's right
                    }
                    break;
            }
        } else{
            m_driveTrain.driveMecanum(0, 0, 0);
        }
    }

    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { //culd use LED to show it's ended
        m_driveTrain.driveMecanum(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // if(Math.abs(tx)<1 && Math.abs(rx)<1 && ta>5){
        //     return true;
        // }
        return false;
    }
}
