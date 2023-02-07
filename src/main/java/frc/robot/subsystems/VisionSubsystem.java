package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

/**
 * Used to interface with the limelight.
 */
public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry hasTarget = table.getEntry("tv");
    private final NetworkTableEntry targetX = table.getEntry("tx"); // Left is positive, right is negative, in degrees
    private final NetworkTableEntry targetY = table.getEntry("ty"); // In degrees
    private final NetworkTableEntry targetArea = table.getEntry("ta"); // Percent area that the target takes up in the whole capture
    private final NetworkTableEntry limelightLED = table.getEntry("ledMode");
    private final NetworkTableEntry pipeline = table.getEntry("pipeline");
    private final NetworkTableEntry tagID = table.getEntry("tid");
    private final NetworkTableEntry botPose = table.getEntry("botpose");

    @Override
    public void periodic() {
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

    public void setPipeline(int pipe) {
        pipeline.setNumber(pipe);
        limelightLED.setNumber(0);
    }

    public int getPipeline() {
        return (int) pipeline.getInteger(0);
    }

    public int getTagID() {
        return (int) tagID.getInteger(0);
    }

    /**
     * Gives the robot pose Translation (x, y, z) & Rotation (x, y, z) in field space. In field space 0, 0, 0 
     * is at the center of the field.
     * @return
     */
    public double[] getBotPose() {
        return botPose.getDoubleArray(new double[0]);
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
     * Calculates distance from the limelight to the selected Target in inches and with the angle given.
     * distance = (h2-h1) / tan(a1+a2)
     * h2 = height of target inches, h1 = height of camera inches, a1 = camera angle degrees, a2 = target angle degrees
     * @return distance in inches
     */
    public double distanceFromTargetInInches(Target target, double angleDeg) {;
        double targetHeight = switch(target) {
            case CUBE -> Vision.cubeHeightInches;
            case CONE -> Vision.coneHeightInches;
            case MID_POLE -> Vision.midPoleHeightInches;
            case HIGH_POLE -> Vision.highPoleHeightInches;
        };
        double radians = Math.toRadians(Vision.cameraAngleMountedDegrees + angleDeg);
        double distance = (targetHeight - Vision.cameraHeightInches) / Math.tan(radians);
        return distance;
    }

    /**
     * Calculates distance from the limelight to the selected Target in inches. Angle is taken from limelight.
     * distance = (h2-h1) / tan(a1+a2)
     * h2 = height of target, h1 = height of camera, a1 = camera angle, a2 = target angle
     * @return distance in inches
     */
    public double distanceFromTargetInInches(Target target) {;
        return distanceFromTargetInInches(target, getTargetY());
    }


    /**
     * Given the distance to the rear pole and the angle between the two poles assuming you are looking at the rear pole 
     * this gives you the angle of how far away you are of the poles being completely in line with each other  
     * @param distanceToPoleInches distance from the limelight to the rear pole in inches
     * @param angleBetweenPolesRadians angle between the two poles in radians
     * @return Theta in radians, or 0 if the result cannot be computed
     */
    public double angleFromLinedUp(double distanceToPoleInches, double angleBetweenPolesRadians) {
        if (distanceToPoleInches <= 0 || angleBetweenPolesRadians == 0) return 0;
        return Math.acos(
            (distanceToPoleInches * Math.tan(angleBetweenPolesRadians)) /
            (Vision.distanceBetweenPolesInches * Math.tan(angleBetweenPolesRadians) + Vision.distanceBetweenPolesInches)
        );
    }

    public enum Target {
        CUBE,
        CONE,
        MID_POLE,
        HIGH_POLE
    }

}
