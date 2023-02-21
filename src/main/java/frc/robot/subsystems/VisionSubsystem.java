package frc.robot.subsystems;


import java.util.ArrayList;
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
    private final IntegerEntry pipeline = mainTable.getIntegerTopic("Current Pipeline").getEntry(0);
    private final NetworkTable coneTable = mainTable.getSubTable("Cone");
    private final NetworkTable cubeTable = mainTable.getSubTable("Cube");
    private final NetworkTable tagTable = mainTable.getSubTable("AprilTag");
    private final List<NetworkTable> subtables = new ArrayList<>();
    private List<PipelineResult> currentResults = new ArrayList<>();

    public VisionSubsystem() {
        super();
        subtables.add(coneTable);
        subtables.add(cubeTable);
        subtables.add(tagTable);
    }
    
    @Override
    public void periodic() {
        // currentResult = new PipelineResult(hasTarget.get(), pitch.get(), yaw.get(), pipeline.get(), orientation.get());
        // System.out.println("Pitch: " + getTargetPitch() + " Yaw: " + getTargetYaw() + " Distance: " + distanceFromTargetInInches(Target.CONE));
        System.out.println(pipeline.get());
    }

    public boolean hasTarget() {
        return false;
        // return currentResult.haveTarget();
    }

    public double getTargetPitch() {
        return 0;
        // if (!hasTarget()) return Double.NaN;
        // return currentResult.pitch();
    }

    public double getTargetYaw() {
        return 0;
        // if (!hasTarget()) return Double.NaN;
        // return currentResult.yaw();
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

    public int getPipeline() {
        // return (int) currentResult.pipeline();
        return 0;
    }

    public void setPipeline(int index) {
        pipeline.set(index);
    }

    public Orientation getConeOrientation() {
        return Orientation.LANDSCAPE;
        // return switch( (int) currentResult.orientation()) {
        //     case 0 -> Orientation.LANDSCAPE;
        //     case 1 -> Orientation.PORTRAIT;
        //     default -> throw new IllegalArgumentException("Invalid number for orientation. - VisionSubsystem");
        // };
    }

    public enum Orientation {
        LANDSCAPE,
        PORTRAIT
    }

    public enum Target {
        NONE,
        CUBE,
        CONE,
        APRILTAG
    }

    private record PipelineResult(
        boolean haveTarget,
        double pitch,
        double yaw,
        long pipeline,
        long orientation
    ) {}

}
