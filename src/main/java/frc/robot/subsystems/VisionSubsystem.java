package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.lib.util.Util;

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
    
    private int i = 0;
    @Override
    public void periodic() {
        currentResults.clear();
        for (NetworkTable table : subtables) {
            currentResults.put(table, new PipelineResult(
                table.getPath(), 
                table.getEntry("Has Target").getBoolean(false), 
                table.getEntry("Pitch").getDouble(0), 
                table.getEntry("Yaw").getDouble(0),
                table.getEntry("tX").getDouble(0),
                table.getEntry("tY").getDouble(0),
                table.getEntry("tZ").getDouble(0),
                table.getEntry("rX").getDouble(0),
                table.getEntry("rY").getDouble(0),
                table.getEntry("rZ").getDouble(0),
                (int) table.getEntry("Pipeline").getInteger(0), 
                (int) table.getEntry("Orientation").getInteger(0)
            ));
        }
        if (i % 2 == 0) {
            Pose3d pose = getPoseToTarget();
            System.out.println("Camera Pose: " + pose + " , Skew: " + getSkew());
        }
        i++;
    }

    private PipelineResult getResult(Target pipeline) {
        return switch(pipeline) {
            case NONE -> throw new IllegalArgumentException("Can't use NONE. - VisionSubsystem");
            case CONE -> currentResults.get(coneTable);
            case CUBE -> currentResults.get(cubeTable);
            case APRILTAG -> currentResults.get(tagTable);
        };
    }

    /**
     * You should always be calling this method before you interact with the vision subsystem for your desired pipeline
     * @param pipeline the pipeline to check for a valid target
     * @return whether or not that pipeline currently has a target
     */
    public boolean hasTarget(Target pipeline) {
        return getResult(pipeline).hasTarget();
    }

    public double getTargetPitch(Target pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return getResult(pipeline).pitch();
    }

    public double getTargetYaw(Target pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return getResult(pipeline).yaw();
    }

    //? AprilTag

    /**
     * @return A Pose3d in meters representing the robot's position in 3d space relative to the target. (0, 0, 0) is the robot's origin.
     */
    public Pose3d getPoseToTarget() {
        if (!hasTarget(Target.APRILTAG)) return null;
        PipelineResult result = getResult(Target.APRILTAG);
        double x = result.tX() + Vision.cameraXOffset;
        double y = result.tY() + Vision.cameraYOffset;
        double z = result.tZ() + Vision.cameraZOffset;
        Vector<N3> rvec = getRotationalVector();
        return new Pose3d(x, y, z, new Rotation3d(rvec));
    }

    /**
     * @return rotational vector of april tag in radians
     */
    public double[] getRotationalArray() {
        if (!hasTarget(Target.APRILTAG)) return null;
        PipelineResult result = getResult(Target.APRILTAG);
        return new double[]{result.rX(), result.rY(), result.rZ()};
    }

    /**
     * @return rotational vector of april tag in radians in wpilib vector form
     */
    public Vector<N3> getRotationalVector() {
        if (!hasTarget(Target.APRILTAG)) return null;
        double[] rvecArray = getRotationalArray();
        Vector<N3> rvec = VecBuilder.fill(rvecArray[0], rvecArray[1], rvecArray[2]); 
        return rvec;
    }
    
    /**
     * @return Skew angle of the april tag in radians
     */
    public double getSkew() {
        if (!hasTarget(Target.APRILTAG)) return Double.NaN;
        Vector<N3> rvec = getRotationalVector();
        Translation3d tagNormal = new Translation3d(0, 0, 1).rotateBy(new Rotation3d(rvec));
        double skew = Util.boundedAngle(Math.atan2(tagNormal.getX(), tagNormal.getZ()) + Math.PI);
        return skew;
    }

    //? AprilTag

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
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Vision.cameraHeightMeters, 
            targetHeightMeters, 
            Vision.cameraPitchRadians, 
            Units.degreesToRadians(getTargetPitch(pipeline))
        );
        return distanceMeters - Vision.cameraZOffset;
    }

    public double getCameraPitch(double distanceMeters, Target pipeline) {
        if (!hasTarget(pipeline)) return Double.NaN;
        return Math.toDegrees(Math.atan((Vision.cameraHeightMeters - 0) / distanceMeters)) - getTargetPitch(pipeline);
    }

    public int getCurrentPipeline() {
        return (int) currentPipeline.get();
    }

    public void setPipeline(Target pipeline) {
        currentPipeline.set(getResult(pipeline).pipeline);
    }

    public Orientation getTargetOrientation(Target pipeline) {
        if (!hasTarget(pipeline)) return null;
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
        boolean hasTarget,
        double pitch,
        double yaw,
        double tX,
        double tY,
        double tZ,
        double rX,
        double rY,
        double rZ,
        int pipeline,
        int orientation
    ) {}

}
