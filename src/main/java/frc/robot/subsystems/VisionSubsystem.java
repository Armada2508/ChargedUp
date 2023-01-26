package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Vision;

/**
 * Used to interface with the limelight.
 */
public class VisionSubsystem {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry hasTarget = table.getEntry("tv");
    private final NetworkTableEntry targetX = table.getEntry("tx"); // Left is positive, right is negative, in degrees
    private final NetworkTableEntry targetY = table.getEntry("ty"); // In degrees
    private final NetworkTableEntry targetArea = table.getEntry("ta"); // Percent area that the target takes up in the whole capture
    private final NetworkTableEntry limelightLED = table.getEntry("ledMode");
    private final NetworkTableEntry pipeline = table.getEntry("pipeline");
    private final NetworkTableEntry pythonData = table.getEntry("llpython");

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

    public double[] getPythonData() {
        return pythonData.getDoubleArray(new double[0]);
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

    /**
     * Calculates distance from the limelight to the selected Target in inches.
     * distance = (h2-h1) / tan(a1+a2)
     * h2 = height of target, h1 = height of camera, a1 = camera angle, a2 = target angle
     * @return distance in inches
     */
    public double distanceFromTargetInInches(Target target) {;
        double targetHeight = switch(target) {
            case CONE -> 0;
            case MID_POLE -> Vision.midPoleHeightInches;
            case HIGH_POLE -> Vision.highPoleHeightInches;
        };
        double radians = Math.toRadians(Vision.cameraAngleMountedDegrees + getTargetY());
        double distance = (targetHeight - Vision.cameraHeightInches) / Math.tan(radians);
        return distance;
    }

    /**
     * Given the distance to the rear pole and the angle between the two poles assuming you are looking at the rear pole 
     * this gives you the angle of how far away you are of the poles being completely in line with each other  
     * @param distanceToRearInches distance from the limelight to the rear pole in inches
     * @param angleBetweenPolesRadians angle between the two poles in radians
     * @return theta in radians
     */
    public double angleFromLinedUp(double distanceToRearInches, double angleBetweenPolesRadians) {
        return Math.acos(
            (distanceToRearInches * Math.tan(angleBetweenPolesRadians)) /
            (Vision.distanceBetweenPolesInches * Math.tan(angleBetweenPolesRadians) + Vision.distanceBetweenPolesInches)
        );
    }

    public enum Target {
        CONE,
        MID_POLE,
        HIGH_POLE
    }

}
