package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    // Network table
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTableEntry limelightTv = table.getEntry("tv");
    private static final NetworkTableEntry limelightTx = table.getEntry("tx"); 
    private static final NetworkTableEntry limelightTy = table.getEntry("ty");
    private static final NetworkTableEntry limelightTa = table.getEntry("ta");


    // Whether the limelight has any valid targets (0 or 1)
    public static boolean hasTarget() {
        return limelightTv.getBoolean(false);
    }


    // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public static double getHorizontalOffset() {
        return limelightTx.getDouble(0.0);
    }


    // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public static double getVerticalOffset() {
        return limelightTy.getDouble(0.0);
    }


    // Target Area (0% of image to 100% of image)  
    public static double getTargetArea() {
        return limelightTa.getDouble(0.0);
    }    
}
