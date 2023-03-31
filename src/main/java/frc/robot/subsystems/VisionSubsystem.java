package frc.robot.subsystems;


import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final CoordinateSystem tagCoordinateSystem = new CoordinateSystem(CoordinateAxis.N(), CoordinateAxis.U(), CoordinateAxis.E());
    private final CoordinateSystem fieldCoordinateSystem = new CoordinateSystem(CoordinateAxis.E(), CoordinateAxis.N(), CoordinateAxis.U());
    private LinearFilter skewAverage = LinearFilter.movingAverage(8);
    private double lastSkew = 0;
    private double currentSkew = 0;
    private AprilTagFieldLayout layout = null;

    public VisionSubsystem() {
        super();
        subtables.add(coneTable);
        subtables.add(cubeTable);
        subtables.add(tagTable);
        try {
            layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            DriverStation.reportError("Unable to open april tag field layout", e.getStackTrace());
        }
    }
    
    private int i = 0;
    @Override
    public void periodic() {
        currentResults.clear();
        for (NetworkTable table : subtables) {
            currentResults.put(table, new PipelineResult(
                table.getPath(), 
                table.getEntry("Has Target").getBoolean(false), 
                table.getEntry("Pitch").getDouble(Double.NaN), 
                table.getEntry("Yaw").getDouble(Double.NaN),
                (int) table.getEntry("Pixel X").getInteger(0),
                table.getEntry("tX").getDouble(Double.NaN),
                table.getEntry("tY").getDouble(Double.NaN),
                table.getEntry("tZ").getDouble(Double.NaN),
                table.getEntry("rX").getDouble(Double.NaN),
                table.getEntry("rY").getDouble(Double.NaN),
                table.getEntry("rZ").getDouble(Double.NaN),
                (int) table.getEntry("id").getInteger(-1),
                (int) table.getEntry("Pipeline").getInteger(-1), 
                (int) table.getEntry("Orientation").getInteger(-1)
            ));
        }
        if (hasTarget(Target.CONE)) {
            if (i % 2 == 0) {
                // System.out.println("X: " + getTargetX(Target.CONE));
            }
        }
        if (hasTarget(Target.APRILTAG)) {
            double skew = computeSkew();
            if (skew != lastSkew) {
                if (Math.toDegrees(Math.abs(currentSkew - skew)) > 3) {
                    // System.out.println("Outlier! " + Math.toDegrees(skew) + " Actual: " + Math.toDegrees(currentSkew));
                } 
                currentSkew = skewAverage.calculate(skew);
                lastSkew = skew;
            }
            if (i % 2 == 0) {
                // Translation3d translation = getTargetPose().getTranslation();
                // System.out.println(translation.getNorm());
                // System.out.println(getTargetPose());
                // System.out.println(getSkew());
                // System.out.println(new Pose3d().relativeTo(getPoseToTarget()));
                // System.out.println(getFieldPose());
            }
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

    public int getTargetX(Target pipeline) {
        if (!hasTarget(pipeline)) return 0;
        return getResult(pipeline).pixelX();
    }

    //? AprilTag

    public Pose3d getFieldPose() {
        if (!hasTarget(Target.APRILTAG)) return null;
        Optional<Pose3d> fieldTagPoseOptional = layout.getTagPose(getResult(Target.APRILTAG).id());
        if (fieldTagPoseOptional.isPresent()) {
            Pose3d fieldTagPose = fieldTagPoseOptional.get();
            Pose3d robotInTagFrame = new Pose3d().relativeTo(getTargetPose());
            Pose3d robotTagFrameCorrected = CoordinateSystem.convert(robotInTagFrame, tagCoordinateSystem, fieldCoordinateSystem);
            Pose3d robotFieldPose = fieldTagPose.transformBy(new Transform3d(new Pose3d(), robotTagFrameCorrected));
            return robotFieldPose;
        }
        return null;
    }

    /**
     * @return A Pose3d in meters representing the target's position in 3d space relative to the robot. (0, 0, 0) is the robot's origin.
     * +X is right, +Y is up, +Z is forward
     */
    public Pose3d getTargetPose() {
        if (!hasTarget(Target.APRILTAG)) return null;
        PipelineResult result = getResult(Target.APRILTAG);
        Vector<N3> rvec = getRotationalVector();
        Translation3d tagTranslation = new Translation3d(result.tX(), result.tY(), result.tZ());
        Transform3d tagPoseCameraFrame = new Transform3d(tagTranslation, new Rotation3d(rvec));
        Pose3d tagPoseRobotFrame = Vision.cameraPoseRobotFrame.transformBy(tagPoseCameraFrame); // no clue why this is flipped
        return tagPoseRobotFrame;
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
     * @return Skew angle of the april tag in radians, counterclockwise positive
     */
    private double computeSkew() {
        if (!hasTarget(Target.APRILTAG)) return Double.NaN;
        Vector<N3> rvec = getRotationalVector();
        Translation3d tagNormal = new Translation3d(0, 0, 1).rotateBy(new Rotation3d(rvec));
        double skew = Util.boundedAngle(Math.atan2(tagNormal.getX(), tagNormal.getZ()) + Math.PI);
        return skew;
    }

    public double getSkew() {
        return currentSkew;
    }

    //? AprilTag

    /**
     * Calculates distance from the center of the robot to the current target in meters
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
        return distanceMeters - Vision.cameraTranslationOffset.getZ();
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
        int pixelX,
        double tX,
        double tY,
        double tZ,
        double rX,
        double rY,
        double rZ,
        int id,
        int pipeline,
        int orientation
    ) {}

}
