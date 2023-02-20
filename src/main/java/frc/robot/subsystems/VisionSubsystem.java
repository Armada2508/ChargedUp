package frc.robot.subsystems;


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

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");
    private final IntegerEntry pipeline = table.getIntegerTopic("Pipeline").getEntry(0);
    // private final IntegerEntry orientation = table.getIntegerTopic("Orientation").getEntry(0);
    // private final BooleanEntry hasTarget = table.getBooleanTopic("HaveTarget").getEntry(false);
    // private final FloatEntry pitch = table.getFloatTopic("Pitch").getEntry(0); // Left is negative, right is positive, in degrees
    // private final FloatEntry yaw = table.getFloatTopic("Yaw").getEntry(0); // In degrees
    private PipelineResult currentResult = new PipelineResult(false, 0, 0, 0, 0);

    public VisionSubsystem() {
        super();
    }
    
    @Override
    public void periodic() {
        // currentResult = new PipelineResult(hasTarget.get(), pitch.get(), yaw.get(), pipeline.get(), orientation.get());
        // System.out.println("Pitch: " + getTargetPitch() + " Yaw: " + getTargetYaw() + " Distance: " + distanceFromTargetInInches(Target.CONE));
    }

    public boolean hasTarget() {
        return currentResult.haveTarget();
    }

    public double getTargetPitch() {
        if (!hasTarget()) return Double.NaN;
        return currentResult.pitch();
    }

    public double getTargetYaw() {
        if (!hasTarget()) return Double.NaN;
        return currentResult.yaw();
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
        return (int) currentResult.pipeline();
    }

    public void setPipeline(int index) {
        pipeline.set(index);
    }

    public Orientation getConeOrientation() {
        return switch( (int) currentResult.orientation()) {
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
