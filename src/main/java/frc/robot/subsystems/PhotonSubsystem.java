package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.VisionSubsystem.Orientation;
import frc.robot.subsystems.VisionSubsystem.Target;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera(Vision.cameraName);
    private PhotonPipelineResult currentResult;

    @Override
    public void periodic() {
        currentResult = camera.getLatestResult();
        if (camera.getPipelineIndex() != Vision.cubePipeline && currentResult.hasTargets()) { /*update to include if its targeted onto an april tag */
            correctConePipeline();
        }
    }

    public boolean hasTargets() {
        return currentResult.hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return hasTargets() ? currentResult.getBestTarget() : null;
    }

    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    /**
     * @param target that you're looking for
     * @return distance to current target in inches or NaN if no targets found.
     */
    public double getDistanceToTargetInches(Target target) {
        if (!hasTargets()) return Double.NaN;
        double targetHeight = switch(target) {
            case NONE -> 0;
            case CUBE -> Vision.cubeHeightInches;
            case CONE -> Vision.coneHeightInches;
            case APRILTAG -> 0; //change
        };
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(Vision.cameraHeightInches), 
            Units.inchesToMeters(targetHeight), 
            Units.degreesToRadians(Vision.cameraAngleMountedDegrees), 
            Units.degreesToRadians(getTargetPitch())
        );
        return Units.metersToInches(distanceMeters) - Vision.distanceToBumperInches;
    }

    public double getTargetPitch() {
        if (!hasTargets()) return Double.NaN;
        return getBestTarget().getPitch();
    }

    public double getTargetYaw() {
        if (!hasTargets()) return Double.NaN;
        return getBestTarget().getYaw();
    }

    /**
     * Assume this method is finished and works
     * @return Landscape when cone is knocked over and portrait when cone is upright
     */
    public Orientation getConeOrientation() {
        return Orientation.LANDSCAPE;
    }

    private Orientation getActualOrientation() {
        List<TargetCorner> corners = getBestTarget().getMinAreaRectCorners();
        double x1 = Math.abs(corners.get(0).x - corners.get(1).x);
        double x2 = Math.abs(corners.get(0).x - corners.get(2).x);
        double y1 = Math.abs(corners.get(0).y - corners.get(1).y);
        double y2 = Math.abs(corners.get(0).y - corners.get(2).y);
        double width = Math.max(x1, x2); // in pixels
        double height = Math.max(y1, y2); // in pixels
        if (height > width) return Orientation.PORTRAIT;
        return Orientation.LANDSCAPE;
    }

    private void correctConePipeline() {
        int index = (getActualOrientation() == Orientation.PORTRAIT) ? Vision.conePipeline : Vision.conePipeline;
        setPipeline(index);
    }

    

}
