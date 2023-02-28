package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private final HashMap<NetworkTable, PipelineResult> currentResults = new HashMap<>();

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
                table.getEntry("X").getDouble(0),
                table.getEntry("Y").getDouble(0),
                table.getEntry("Z").getDouble(0),
                (int) table.getEntry("Pipeline").getInteger(0), 
                (int) table.getEntry("Orientation").getInteger(0)
            ));
        }
        // System.out.println((getPoseToTarget(Target.APRILTAG)));
        // distanceFromTargetMeters(Target.CUBE);
        // System.out.println("Pitch: " + getTargetPitch(Target.CUBE) + " Yaw: " + getTargetYaw(Target.CUBE) + " Distance: " + distanceFromTargetMeters(Target.CUBE));
    }

    private PipelineResult getResult(Target pipeline) {
        return switch(pipeline) {
            case NONE -> throw new IllegalArgumentException("Can't use NONE. - VisionSubsystem");
            case CONE -> currentResults.get(coneTable);
            case CUBE -> currentResults.get(cubeTable);
            case APRILTAG -> currentResults.get(tagTable);
        };
    }

    public boolean hasTarget(Target pipeline) {
        return getResult(pipeline).haveTarget();
    }

    public double getTargetPitch(Target pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return getResult(pipeline).pitch();
    }

    public double getTargetYaw(Target pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return getResult(pipeline).yaw();
    }

    /**
     * @param pipeline to use for getting target pose
     * @return A Pose2d in meters representing the robot's position in 2d space relative to the target at (0, 0)
     */
    public Pose2d getPoseToTarget(Target pipeline) {
        if (!hasTarget(pipeline)) return new Pose2d();
        double x = getResult(pipeline).x();
        double z = getResult(pipeline).z();
        double yaw = getResult(pipeline).yaw();
        return new Pose2d(x, z, Rotation2d.fromDegrees(yaw));
    }

    /**
     * Calculates distance from the front of the robot to the current target in meters
     * distance = (h2-h1) / tan(a1+a2)
     * h2 = height of target meters, h1 = height of camera meters, a1 = camera angle degrees, a2 = target angle degrees
     * @return distance in meters
     */
    public double distanceFromTargetMeters(Target pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        double targetHeightMeters = switch(pipeline) {
            case NONE -> 0;
            case CONE -> Vision.coneHeightMeters;
            case CUBE -> Vision.cubeHeightMeters;
            case APRILTAG -> Vision.aprilTagHeightMeters; 
        };
        // System.out.println(Vision.cameraHeightMeters + " " + targetHeightMeters + " " + Vision.mountedCameraAngleRad);
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Vision.cameraHeightMeters, 
            targetHeightMeters, 
            Vision.cameraPitchRadians, 
            Units.degreesToRadians(getTargetPitch(pipeline))
        );
        return distanceMeters - Vision.distanceToBumperMeters;
    }

    public int getCurrentPipeline() {
        return (int) currentPipeline.get();
    }

    public void setPipeline(Target pipeline) {
        currentPipeline.set(getResult(pipeline).pipeline);
    }

    public Orientation getTargetOrientation(Target pipeline) {
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

    public enum Target {
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
        double x,
        double y,
        double z,
        int pipeline,
        int orientation
    ) {}

}
