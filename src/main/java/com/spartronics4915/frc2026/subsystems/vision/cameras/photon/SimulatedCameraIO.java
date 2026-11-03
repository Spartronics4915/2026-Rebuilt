package com.spartronics4915.frc2026.subsystems.vision.cameras.photon;

import com.spartronics4915.frc2026.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** PhotonVision camera backed by PhotonLib's field and camera simulation. */
public class SimulatedCameraIO extends PhotonCameraIO {

    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;

    public SimulatedCameraIO(
        CameraConfig config,
        AprilTagFieldLayout fieldLayout,
        VisionSystemSim visionSim,
        SimCameraProperties cameraProperties
    ) {
        super(config, fieldLayout, new PhotonCamera(config.name));
        this.visionSim = visionSim;
        this.cameraSim = new PhotonCameraSim(photonCamera, cameraProperties);

        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);
        visionSim.addCamera(cameraSim, config.getCurrentTransform());
    }

    public SimulatedCameraIO(
            CameraConfig config,
            AprilTagFieldLayout fieldLayout,
            VisionSystemSim visionSim
    ) {
        this(
            config,
            fieldLayout,
            visionSim,
            VisionConstants.createSimulationCameraProperties()
        );
    }

    public void updateSimulationTransform() {
        if (config.isDynamic()) {
            visionSim.adjustCamera(cameraSim, config.getCurrentTransform());
        }
    }
}