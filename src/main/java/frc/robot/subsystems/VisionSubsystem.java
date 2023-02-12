package frc.robot.subsystems;


import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.PhotonSubsystem.Target;

/**
 * Used to interface with the raspberry pi.
 */
public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
    private final NetworkTableValue hasTarget = table.getValue("HasTarget");
    private final NetworkTableValue pitch = table.getValue("Pitch"); // Left is negative, right is positive, in degrees
    private final NetworkTableValue yaw = table.getValue("Yaw"); // In degrees

    public VisionSubsystem() {
        super();
    }
    
    @Override
    public void periodic() {
        // System.out.println("Pitch: " + getTargetPitch() + " Yaw: " + getTargetYaw());
    }

    public boolean hasTarget() {
        return hasTarget.getBoolean();
    }

    public double getTargetPitch() {
        if (!hasTarget()) return Double.NaN;
        return pitch.getDouble();
    }

    public double getTargetYaw() {
        if (!hasTarget()) return Double.NaN;
        return yaw.getDouble();
    }

    /**
     * Calculates distance from the limelight to the selected Target in inches and with the angle given.
     * distance = (h2-h1) / tan(a1+a2)
     * h2 = height of target inches, h1 = height of camera inches, a1 = camera angle degrees, a2 = target angle degrees
     * @return distance in inches
     */
    public double distanceFromTargetInInches(Target target) {;
        if (!hasTarget()) return Double.NaN;
        double targetHeight = switch(target) {
            case NONE -> 0;
            case CUBE -> Vision.cubeHeightInches;
            case CONE -> Vision.coneHeightInches;
            case APRILTAG -> Vision.aprilTagHeightInches; 
        };
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(Vision.cameraHeightInches), 
            Units.inchesToMeters(targetHeight), 
            Units.degreesToRadians(Vision.cameraAngleMountedDegrees), 
            Units.degreesToRadians(getTargetPitch())
        );
        return Units.metersToInches(distanceMeters) - Vision.distanceToBumperInches;
    }

}
