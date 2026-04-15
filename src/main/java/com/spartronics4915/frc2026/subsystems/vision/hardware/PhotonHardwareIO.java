package com.spartronics4915.frc2026.subsystems.vision.hardware;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.RobotState;
import com.spartronics4915.frc2026.subsystems.vision.samples.FiducialObservation;
import com.spartronics4915.frc2026.subsystems.vision.samples.PoseEstimate;

public class PhotonHardwareIO implements VisionIO {
    private final PhotonCamera cameraA;
    private final PhotonCamera cameraB;

    private final PhotonPoseEstimator poseEstimatorA;
    private final PhotonPoseEstimator poseEstimatorB;

    private final RobotState robotState;
    private final AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

    public PhotonHardwareIO(RobotState robotState) {
        this.robotState = robotState;

        this.cameraA = new PhotonCamera(cameraATableName);
        this.cameraB = new PhotonCamera(cameraBTableName);

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Transformations using the degrees provided in constants
        Transform3d robotToCameraA = new Transform3d(
            new Translation3d(robotToCameraAForward, robotToCameraASide, cameraAHeightOffGroundMeters),
            new Rotation3d(0.0, Math.toRadians(-cameraAPitchDegrees), cameraAYawOffset.getRadians())
        );

        Transform3d robotToCameraB = new Transform3d(
            new Translation3d(robotToCameraBForward, robotToCameraBSide, cameraBHeightOffGroundMeters),
            new Rotation3d(0.0, Math.toRadians(-cameraBPitchDegrees), cameraBYawOffset.getRadians())
        );

        this.poseEstimatorA = new PhotonPoseEstimator(fieldLayout, robotToCameraA);
        this.poseEstimatorB = new PhotonPoseEstimator(fieldLayout, robotToCameraB);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        updateCameraInputs(inputs.cameraA, cameraA, poseEstimatorA);
        updateCameraInputs(inputs.cameraB, cameraB, poseEstimatorB);

        latestInputs.set(inputs);
    }

    private void updateCameraInputs(VisionIOInputs.CameraInputs inputs, PhotonCamera camera, PhotonPoseEstimator estimator) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        
        if (results.isEmpty()) {
            inputs.seesTarget = false;
            return;
        }

        // Get the most recent result from the queue
        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        inputs.seesTarget = latestResult.hasTargets();

        if (inputs.seesTarget) {
            Optional<EstimatedRobotPose> poseOptional = (latestResult.targets.size() > 1) 
                ? estimator.estimateCoprocMultiTagPose(latestResult) 
                : estimator.estimateLowestAmbiguityPose(latestResult);

            if (poseOptional.isPresent()) {
                EstimatedRobotPose estimatedPose = poseOptional.get();
                
                // Use your Record's static factory method
                inputs.poseEstimate = PoseEstimate.fromPhotonCamera(estimatedPose);
                
                // Also update the raw fields in VisionIOInputs if necessary
                inputs.pose3d = estimatedPose.estimatedPose;
                inputs.poseCount = estimatedPose.targetsUsed.size();
                
                // Update fiducial observations (Assuming a helper exists for Photon targets)
                inputs.fiducialObservations = FiducialObservation.fromPhotonCamera(estimatedPose.targetsUsed);
            }
        }
    }

}