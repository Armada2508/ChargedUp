package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.VisionSubsystem.Target;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera(Vision.cameraName);

    @Override
    public void periodic() {
        if (hasTargets()) {
            System.out.println(getDistanceToTargetInches(Target.CONE));
        }
    }

    private PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getLatestResult().getBestTarget();
    }

    /**
     * @param target that you're looking for
     * @return distance to current target in inches or NaN if no targets found.
     */
    public double getDistanceToTargetInches(Target target) {
        if (!hasTargets()) return Double.NaN;
        double targetHeight = switch(target) {
            case CONE -> Vision.coneHeightInches;
            case MID_POLE -> Vision.midPoleHeightInches;
            case HIGH_POLE -> Vision.highPoleHeightInches;
        };
        double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(Vision.cameraHeightInches), 
            Units.inchesToMeters(targetHeight), 
            Units.degreesToRadians(Vision.cameraAngleMountedDegrees), 
            Units.degreesToRadians(getBestTarget().getPitch())
        );
        return Units.metersToInches(distanceMeters);
    }

    public double getTargetPitch() {
        if (!hasTargets()) return 0;
        return getBestTarget().getPitch();
    }

    public double getTargetYaw() {
        if (!hasTargets()) return 0;
        return getBestTarget().getYaw();
    }

}
