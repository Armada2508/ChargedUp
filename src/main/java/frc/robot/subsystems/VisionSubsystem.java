package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Vision;

/**
 * Used to interface with the limelight.
 */
public class VisionSubsystem {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry hasTarget = table.getEntry("tv");
    private NetworkTableEntry targetX = table.getEntry("tx"); // Left is positive, right is negative, in degrees
    private NetworkTableEntry targetY = table.getEntry("ty"); // In degrees
    private NetworkTableEntry targetArea = table.getEntry("ta"); // Percent area that the target takes up in the whole capture
    private NetworkTableEntry limelightLED = table.getEntry("ledMode");
    private NetworkTableEntry pipeline = table.getEntry("pipeline");

    public VisionSubsystem() {
        setPipeline(Vision.colorPipeline);
    }

    public boolean hasTarget() {
        return hasTarget.getInteger(0) == 1;
    }

    public double getTargetX() {
        return targetX.getDouble(0);
    }

    public double getTargetY() {
        return targetY.getDouble(0);
    }

    public double getTargetArea() {
        return targetArea.getDouble(0);
    }

    /**
     * h2 = height of target, h1 = height of camera, a1 = camera angle, a2 = target angle
     * @return distance in inches
     */
    public double distanceFromTargetInInches(Target target) {;
        double targetHeight = switch(target) {
            case CONE -> 0;
            case MID_POLE -> Vision.midPoleHeight;
            case HIGH_POLE -> Vision.highPoleHeight;
        };
        double radians = Math.toRadians(Vision.cameraAngle + getTargetY());
        double distance = (targetHeight - Vision.cameraHeight) / Math.tan(radians);
        return distance;
        // d = (h2-h1) / tan(a1+a2)
    }

    public void setPipeline(int pipe) {
        pipeline.setNumber(pipe);
        limelightLED.setNumber(0);
    }

    public int getPipeline() {
        return (int) pipeline.getInteger(0);
    }

    public void limelightON() {
        limelightLED.setNumber(3);
    }

    public void limelightOFF() {
        limelightLED.setNumber(1);
    }

    public boolean isLimelightON() {
        return limelightLED.getInteger(0) == 3;
    }

    public enum Target {
        CONE,
        MID_POLE,
        HIGH_POLE
    }

}
