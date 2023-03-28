package frc.robot.helpers;

public class PathPlannerPath {
    public PathPlannerPath(String name, boolean resetOdometry, double maxVelocity, double maxAcceleration) {
        this.name = name;
        this.resetOdometry = resetOdometry;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public final String name;
    public final boolean resetOdometry;
    public final double maxVelocity;
    public final double maxAcceleration;
}