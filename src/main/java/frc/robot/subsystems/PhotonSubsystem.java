package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.Vision;

public class PhotonSubsystem {

    private final PhotonCamera camera = new PhotonCamera(Vision.cameraName);

    private PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getLatestResult().getBestTarget();
    }

}
