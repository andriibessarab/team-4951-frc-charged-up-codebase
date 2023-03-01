package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


public final class LimelightVision implements Sendable {
    public final class LimelightConstants {
        public final static int kReflectiveTapePipeline = 1;
        public final static int kAprilTagsPipeline = 0;

        // Approximate(need to be measured)
        public final static int MAX_VERT_OFFSET_FOR_LOW = 10;
        public final static int HEIGHT_TO_LOW = 24;
        public final static int MAX_VERT_OFFSET_FOR_MED = 20;
        public final static int HEIGHT_TO_MED = 36;
        public final static int MAX_VERT_OFFSET_FOR_HIGH = 30;
        public final static int HEIGHT_TO_HIGH = 48;
    }


    /**
     * Provides an object through which to access the networkTables entries associated with the limelight
     */
    private NetworkTable limelightTable;


    /**
     * Creates a new instance of the LimelightVision class.
     *
     * @param hostName The name of the host where the Limelight table is stored.
     * 
     * @see <a href="https://docs.limelightvision.io/en/latest/networktables_api.html">Limelight NetworkTables API</a>
     */
    public LimelightVision(String hostName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(hostName);
    }


    /**
     * Creates a number Array to store the robot position values from the botPose entry
     * Fills it with the numbers from the defaultValues array
     */
    private final Number[] getRobotPosition3D()
    {
        return limelightTable.getEntry("botPose").getNumberArray(new Number[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    }


    /**
     * Gets the current pipeline of the limelight
     * @return returns which pipeline is currently used. returns -1 if no pipeline selected.
     */
    public final int getPipeline() {
        return limelightTable.getEntry("pipeline").getNumber(-1).intValue();
    }


    /**
     * Allows us to set the vision pipeline to a number of our choosing
     */
    public final void setPipeline(int desiredPipeline) {
        limelightTable.getEntry("pipeline").setNumber(desiredPipeline);
    }


    /** 
     * Switch between two pipelines(i.o.
     * for reflective tape and april tags) 
     */
    public final void togglePipeline() {
        setPipeline((getPipeline() == 0) ? 1 : 0);
    } 


    /**
     * change the led mode of the limelight
     * @param mode 0 for pipeline default, 1 for off, 2 for blink, 3 for on
     */
    public final void setLedMode(int mode) {
        limelightTable.getEntry("ledMode").setInteger(mode);
    }


    /**
      * returns a boolean value that lets us know if the limelight has any targets
      * @return If the limelight has any targets
      */
    public final boolean hasTargets() {
        return limelightTable.getEntry("tv").getBoolean(false);
    }


    /**
      * Returns the total area that the current target takes up on the limelight's screen
      * Outputs a value associated with the percent of the screen being taken up
      * @return The area of the limelight screen being taken up
      */
    public final Double getTargetArea() {
        return hasTargets() ? limelightTable.getEntry("ta").getNumber(0).doubleValue() : Double.NaN;
    }


    /**
      *checks if the limelight is in Apriltag, and if it has a target,  returns the ID of the Apriltag. Otherwise, it returns null.
      * @return The ID of the currently targeted Apriltag
      */
    public final Double getTargetID() {
        if(getPipeline() == LimelightConstants.kAprilTagsPipeline && hasTargets()) {
            return limelightTable.getEntry("tid").getDouble(0.0);
        }

        return null;

    }


    /**
     * Gets the horizontal offset angle from networkTable
     * @return The horizontal offset angle from the limelight crosshair to the target
     */
    public final Double getHorizontalOffset() {  
        return hasTargets() ? limelightTable.getEntry("tx").getNumber(0).doubleValue() : Double.NaN;
    }

   
    /**
     * Gets the vertical offset angle from the limelight to the target
     * @return The vertical angle from the limelight crosshair to the target
     */
    public final Double getVerticalOffset() {
        return hasTargets() ? limelightTable.getEntry("tx").getNumber(0).doubleValue() : Double.NaN;
    }


    /**
      * Outputs between -90 and 0 degrees
      * @return The skew of the target that is currently within the limelight's viewframe
      */
    public final Double getSkew() {
        return hasTargets() ? limelightTable.getEntry("ts0").getNumber(0).doubleValue() : Double.NaN;
    }   


    /**
     * Uses the horizontal offset that determines if the robot is looking left
     * @return A boolean that says if the robot is looking left
     */
    public final boolean isLookingLeft() {

        return getHorizontalOffset() < 0;

    }


    /**
     * Uses the pitch and yaw values from networkTables to construct and returns a rotation2D
     * @return A rotation2D made from the robot pitch and yaw values, represents rotation to the currently seen target
     */
    public final Rotation2d createRotation2D() {

        /**
         * Gets the pitch value from the robotPositionValues array & converts it to radians
         */
        double robotRotationPitch = getRobotPosition3D()[3].doubleValue();
        double robotRotationPitchRadians = Math.toRadians(robotRotationPitch);

        /**
         * Gets the yaw value from the robotPositionValues array & converts it to radians
         */
        double robotRotationYaw = getRobotPosition3D()[4].doubleValue();
        double robotRotationYawRadians = Math.toRadians(robotRotationYaw);

        /**
         * Uses the pitch & yaw value to construct a rotation2d, then returns it
         */
        return new Rotation2d(robotRotationPitchRadians, robotRotationYawRadians);

    }


    /**
     * Uses values from networkTables to construct and return a translation2D
     * @return a translation2D made from the robot x and robot y values, represents movement to the currently seen target
     */
    public final Translation2d createTranslation2D() {
        
        /**
         * Gets the x & y values from the robotPositionValues array
         */
        double robotPositionX = getRobotPosition3D()[0].doubleValue();
        double robotPositionY = getRobotPosition3D()[1].doubleValue();

        /**
         * Uses the x & y values to construct a translation2d, then returns it
         */
        return new Translation2d(robotPositionX, robotPositionY);

    }


    /**
     *  Uses an existing translation2d and rotation2d to make a transform2d
     * @param constructorTranslation2d
     * @param constructorRotation2d
     * @return A transform2d (rotates and drives at the same time)
     */
    public final Transform2d createTransform2D(Translation2d constructorTranslation2d, Rotation2d constructorRotation2d) {
        /**
         * Uses a translation2d & a rotation2d parameter to construct a transform2d, then returns it
         */
        return new Transform2d(constructorTranslation2d, constructorRotation2d);
    }


    /**
     * Checks what target we are looking at, calculates the forward distance, and returns it
     * @return Forward distance from the current vision target
     */
    public final Double forwardDistanceToTarget() {
        if(hasTargets()){
            double verticalOffset = getVerticalOffset();

            if(verticalOffset > 0 && verticalOffset <= LimelightConstants.MAX_VERT_OFFSET_FOR_LOW){
                double horizontalFromLow = LimelightConstants.HEIGHT_TO_LOW / Math.tan(verticalOffset);
                return horizontalFromLow;
            }

            if(verticalOffset > LimelightConstants.MAX_VERT_OFFSET_FOR_LOW && verticalOffset <= LimelightConstants.MAX_VERT_OFFSET_FOR_MED){
                double horizontalFromMed = LimelightConstants.HEIGHT_TO_MED / Math.tan(verticalOffset);
                return horizontalFromMed;
            }
            
            if(verticalOffset > LimelightConstants.MAX_VERT_OFFSET_FOR_MED && verticalOffset <= LimelightConstants.MAX_VERT_OFFSET_FOR_HIGH){
                double horizontalFromHigh = LimelightConstants.HEIGHT_TO_HIGH / Math.tan(verticalOffset);
                return horizontalFromHigh;
            }
        }
        return Double.NaN;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Limelight");
        builder.addBooleanProperty("Has Targets", this::hasTargets, null);
        builder.addDoubleProperty("Horizontal Offset", this::getHorizontalOffset, null);
        builder.addDoubleProperty("Vertical Offset", this::getVerticalOffset, null);
        builder.addDoubleProperty("Forward Distance From Target", this::forwardDistanceToTarget, null);
        builder.addIntegerProperty("Current Pipeline", this::getPipeline, null);
    }
}
