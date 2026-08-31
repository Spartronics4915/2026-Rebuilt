package com.spartronics4915.frc2026.subsystems.vision.cameras.photon;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2026.subsystems.vision.cameras.CameraIO;
import com.spartronics4915.frc2026.util.vision.VisionEstimate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** PhotonVision-backed AprilTag camera. Produces every valid unread observation. */
public class PhotonCameraIO extends CameraIO {

    protected final PhotonCamera photonCamera;
    protected final PhotonPoseEstimator photonPoseEstimator;
    protected final AprilTagFieldLayout fieldLayout;

    public PhotonCameraIO(CameraConfig config, AprilTagFieldLayout fieldLayout) {
        this(config, fieldLayout, new PhotonCamera(config.name));
    }

    protected PhotonCameraIO(
        CameraConfig config,
        AprilTagFieldLayout fieldLayout,
        PhotonCamera photonCamera
    ) {
        super(config);
        this.fieldLayout = fieldLayout;
        this.photonCamera = photonCamera;
        this.photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, config.getCurrentTransform());
    }

    @Override
    protected List<VisionEstimate> readEstimates() {
        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return List.of();
        }

        List<VisionEstimate> estimates = new ArrayList<>();
        for (PhotonPipelineResult result : results) {
            if (result.getTargets().isEmpty()) {
                continue;
            }

            Optional<EstimatedRobotPose> estimate = estimatePose(result);
            if (estimate.isPresent()) {
                VisionEstimate filteredEstimate = toFilteredVisionEstimate(estimate.get(), result);
                if (filteredEstimate != null) {
                    estimates.add(filteredEstimate);
                }
            }
        }

        return estimates;
    }

    private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
        if (config.isDynamic()) {
            Transform3d transform = config.getTransformAtTimestamp(result.getTimestampSeconds());
            photonPoseEstimator.setRobotToCameraTransform(transform);
        }

        Optional<EstimatedRobotPose> estimate = photonPoseEstimator
            .estimateCoprocMultiTagPose(result);

        if (estimate.isEmpty()) {
            estimate = photonPoseEstimator.estimateLowestAmbiguityPose(result);
        }

        return estimate;
    }

    private VisionEstimate toFilteredVisionEstimate(
        EstimatedRobotPose estimate,
        PhotonPipelineResult result
    ) {
        if (estimate == null || estimate.targetsUsed.isEmpty()) {
            return null;
        }

        Pose3d pose = estimate.estimatedPose;
        int[] tagIds = getTagIds(estimate.targetsUsed);
        boolean multiTag = tagIds.length >= 2;
        double distance = averageTagDistance(estimate, tagIds);
        double ambiguity = multiTag ? 0.0 : averageAmbiguity(estimate.targetsUsed);

        if (pose.getZ() < MIN_ROBOT_Z_METERS
                || pose.getZ() > MAX_ROBOT_Z_METERS
                || !Double.isFinite(distance)
                || distance > MAX_AVERAGE_TAG_DISTANCE_METERS
                || (!multiTag && (!Double.isFinite(ambiguity) || ambiguity > MAX_SINGLE_TAG_AMBIGUITY))
                || !Double.isFinite(estimate.timestampSeconds)
                || estimate.timestampSeconds <= 0.0
                || !Double.isFinite(result.metadata.getLatencyMillis())) {
            return null;
        }

        double latencySeconds = result.metadata.getLatencyMillis() / 1000.0;

        return new VisionEstimate(
            tagIds,
            estimate.estimatedPose,
            Seconds.of(estimate.timestampSeconds),
            distance,
            ambiguity,
            tagSpanMeters(tagIds),
            latencySeconds,
            multiTag || USE_VISION_ROTATION_FOR_SINGLE_TAG);
    }

    private int[] getTagIds(List<PhotonTrackedTarget> targets) {
        int[] tagIds = new int[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            tagIds[i] = targets.get(i).getFiducialId();
        }
        return tagIds;
    }

    /** Calculates camera-to-tag distance from field geometry, which also works for MultiTag results. */
    private double averageTagDistance(EstimatedRobotPose estimate, int[] tagIds) {
        Transform3d robotToCamera = config.getTransformAtTimestamp(estimate.timestampSeconds);
        Pose3d cameraPose = estimate.estimatedPose.transformBy(robotToCamera);

        double totalDistance = 0.0;
        int validTags = 0;

        for (int tagId : tagIds) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
            if (tagPose.isEmpty()) {
                continue;
            }

            totalDistance += tagPose.get().getTranslation().getDistance(cameraPose.getTranslation());
            validTags++;
        }

        return validTags == 0 ? Double.POSITIVE_INFINITY : totalDistance / validTags;
    }

    private double averageAmbiguity(List<PhotonTrackedTarget> targets) {
        double sum = 0.0;
        int count = 0;
        for (PhotonTrackedTarget target : targets) {
            double ambiguity = target.getPoseAmbiguity();
            if (Double.isFinite(ambiguity) && ambiguity >= 0.0) {
                sum += ambiguity;
                count++;
            }
        }
        return count == 0 ? Double.NaN : sum / count;
    }

    private double tagSpanMeters(int[] tagIds) {
        double maxSpan = 0.0;

        for (int i = 0; i < tagIds.length; i++) {
            Optional<Pose3d> first = fieldLayout.getTagPose(tagIds[i]);
            if (first.isEmpty()) {
                continue;
            }

            for (int j = i + 1; j < tagIds.length; j++) {
                Optional<Pose3d> second = fieldLayout.getTagPose(tagIds[j]);
                if (second.isEmpty()) {
                    continue;
                }

                maxSpan = Math.max(
                    maxSpan,
                    first.get().getTranslation().toTranslation2d()
                        .getDistance(second.get().getTranslation().toTranslation2d()));
            }
        }

        return maxSpan;
    }

    @Override
    protected void applyPipeline(CameraPipeline pipeline) {
        photonCamera.setPipelineIndex(pipeline.index());
    }
}
