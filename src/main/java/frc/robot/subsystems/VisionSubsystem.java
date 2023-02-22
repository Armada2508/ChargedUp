package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

/**
 * Used to interface with the raspberry pi.
 */
public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable mainTable = NetworkTableInstance.getDefault().getTable("VisionRPI");
    private final IntegerEntry currentPipeline = mainTable.getIntegerTopic("Current Pipeline").getEntry(0);
    private final NetworkTable coneTable = mainTable.getSubTable("Cone");
    private final NetworkTable cubeTable = mainTable.getSubTable("Cube");
    private final NetworkTable tagTable = mainTable.getSubTable("AprilTag");
    private final List<NetworkTable> subtables = new ArrayList<>();
    private HashMap<NetworkTable, PipelineResult> currentResults = new HashMap<>();

    public VisionSubsystem() {
        super();
        subtables.add(coneTable);
        subtables.add(cubeTable);
        subtables.add(tagTable);
    }
    
    @Override
    public void periodic() {
        currentResults.clear();
        for (NetworkTable table : subtables) {
            currentResults.put(table, new PipelineResult(
                table.getPath(), 
                table.getEntry("Has Target").getBoolean(false), 
                table.getEntry("Pitch").getDouble(0), 
                table.getEntry("Yaw").getDouble(0),
                (int) table.getEntry("Pipeline").getInteger(0), 
                (int) table.getEntry("Orientation").getInteger(0)
            ));
        }
        // System.out.println("Pitch: " + getTargetPitch(Pipeline.CUBE) + " Yaw: " + getTargetYaw(Pipeline.CUBE) + " Distance: " + distanceFromTargetInInches(Pipeline.CUBE));
    }

    private PipelineResult getResult(Pipeline pipeline) {
        return switch(pipeline) {
            case NONE -> throw new IllegalArgumentException("Can't use NONE.");
            case CONE -> currentResults.get(coneTable);
            case CUBE -> currentResults.get(cubeTable);
            case APRILTAG -> currentResults.get(tagTable);
        };
    }

    public boolean hasTarget(Pipeline pipeline) {
        return getResult(pipeline).haveTarget();
    }

    public double getTargetPitch(Pipeline pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return getResult(pipeline).pitch();
    }

    public double getTargetYaw(Pipeline pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return getResult(pipeline).yaw();
    }

    /**
     * Calculates distance from the front of the robot to the current target in inches and with the angle given.
     * distance = (h2-h1) / tan(a1+a2)
     * h2 = height of target inches, h1 = height of camera inches, a1 = camera angle degrees, a2 = target angle degrees
     * @return distance in inches
     */
    public double distanceFromTargetInInches(Pipeline pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        double targetHeight = switch(pipeline) {
            case NONE -> 0;
            case CONE -> Vision.coneHeightInches;
            case CUBE -> Vision.cubeHeightInches;
            case APRILTAG -> Vision.aprilTagHeightInches; 
        };
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(Vision.cameraHeightInches), 
            Units.inchesToMeters(targetHeight), 
            Units.degreesToRadians(Vision.cameraAngleMountedDegrees), 
            Units.degreesToRadians(getTargetPitch(pipeline))
        );
        return Units.metersToInches(distanceMeters) - Vision.distanceToBumperInches;
    }

    public int getCurrentPipeline() {
        return (int) currentPipeline.get();
    }

    public void setPipeline(Pipeline pipeline) {
        currentPipeline.set(getResult(pipeline).pipeline);
    }

    public Orientation getTargetOrientation(Pipeline pipeline) {
        return switch(getResult(pipeline).orientation()) {
            case 0 -> Orientation.LANDSCAPE;
            case 1 -> Orientation.PORTRAIT;
            default -> throw new IllegalArgumentException("Invalid number for orientation. - VisionSubsystem");
        };
    }

    public enum Orientation {
        LANDSCAPE,
        PORTRAIT
    }

    public enum Pipeline {
        NONE,
        CONE,
        CUBE,
        APRILTAG
    }

    private record PipelineResult(
        String name,
        boolean haveTarget,
        double pitch,
        double yaw,
        int pipeline,
        int orientation
    ) {}

}
