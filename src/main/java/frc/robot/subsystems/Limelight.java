package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public final class Limelight {
    // Network table
    private final static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final static NetworkTableEntry limelightTv = table.getEntry("tv");
    private final static NetworkTableEntry limelightTx = table.getEntry("tx"); 
    private final static NetworkTableEntry limelightTy = table.getEntry("ty");
    private final static NetworkTableEntry limelightTa = table.getEntry("ta");

    // Whether the limelight has any valid targets (0 or 1)
    public final static boolean hasTarget() {
        return limelightTv.getBoolean(false);
    }

    // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    public final static double getHorizontalOffset() {
        return limelightTx.getDouble(0.0);
    }

    // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    public final static double getVerticalOffset() {
        return limelightTy.getDouble(0.0);
    }

    // Target Area (0% of image to 100% of image)  
    public final static double getTargetArea() {
        return limelightTa.getDouble(0.0);
    }    
}
