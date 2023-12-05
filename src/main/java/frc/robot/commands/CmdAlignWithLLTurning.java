package frc.robot.Commands;

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


        // double[] botPose = LimelightHelpers.getBotPose(limelightName);
        // Translation2d tran2d = new Translation2d(botPose[0], botPose[1]);
        // Rotation2d r2d = new Rotation2d(Units.degreesToRadians(botPose[5]));

        // Pose3d botPose = LimelightHelpers.getCameraPose3d_TargetSpace(limelightName);


        tx = LimelightHelpers.getTX(limelightName);
        ta = LimelightHelpers.getTA(limelightName);
        // rx = botPose.getRotation().getX();
        SmartDashboard.putNumber("X", LimelightHelpers.getTX(limelightName));
        // SmartDashboard.putNumber("rX", r2d.getDegrees());
        if(targetDetected) {
            switch (caseNumber){
                case 1: //rotate to face april tag, use rx
                    caseNumber = 2;
                    break;
                case 2: //strafe to be infront of april tag, use tx
                    if(tx>1){
                        m_driveTrain.driveMecanum(0.3, 0, 0);
                    } else if(tx<-1){
                        m_driveTrain.driveMecanum(-0.3, 0, 0);
                    } else{
                        m_driveTrain.driveMecanum(0, 0, 0);
                        caseNumber=3;
                    }
                    break;
                case 3: //drive to be at right distance, use ta(or something better)
                    if(ta<10){ //play with this value
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
