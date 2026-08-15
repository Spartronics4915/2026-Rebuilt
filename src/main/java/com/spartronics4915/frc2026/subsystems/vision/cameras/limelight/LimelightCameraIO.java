package com.spartronics4915.frc2026.subsystems.vision.cameras.limelight;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.vision.cameras.CameraIO;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.PoseEstimate;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.RawFiducial;
import com.spartronics4915.frc2026.util.vision.VisionEstimate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Limelight MegaTag1 backend. */
public class LimelightCameraIO extends CameraIO {

    private final AprilTagFieldLayout fieldLayout;
    private double lastTimestampSeconds = Double.NEGATIVE_INFINITY;

    public LimelightCameraIO(CameraConfig config, AprilTagFieldLayout fieldLayout) {
        super(config);
        this.fieldLayout = fieldLayout;
    }

    @Override
    protected List<VisionEstimate> readEstimates() {
        // MegaTag1 uses the camera pose currently configured in Limelight. This is therefore the
        // latest available transform, not a historical transform for an already-solved frame.
        configureCameraPose(config.getCurrentTransform());

        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);
        if (estimate == null || estimate.timestampSeconds <= lastTimestampSeconds) {
            return List.of();
        }

        lastTimestampSeconds = estimate.timestampSeconds;

        return filterPoseEstimate(estimate)
            .map(this::toVisionEstimate)
            .map(List::of)
            .orElseGet(List::of);
    }

    private void configureCameraPose(Transform3d transform) {
        if (!Robot.isReal()) {
            return;
        }

        Translation3d translation = transform.getTranslation();
        Rotation3d rotation = transform.getRotation();

        LimelightHelpers.setCameraPose_RobotSpace(
            config.name,
            translation.getX(),
            translation.getY(),
            translation.getZ(),
            Math.toDegrees(rotation.getX()),
            Math.toDegrees(rotation.getY()),
            Math.toDegrees(rotation.getZ()));
    }

    private Optional<PoseEstimate> filterPoseEstimate(PoseEstimate estimate) {
        if (estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
            return Optional.empty();
        }

        boolean multiTag = estimate.tagCount >= 2;
        double ambiguity = multiTag ? 0.0 : avgAmbiguity(estimate);

        if (!Double.isFinite(estimate.avgTagDist)
                || estimate.avgTagDist > MAX_AVERAGE_TAG_DISTANCE_METERS
                || !Double.isFinite(estimate.latency)
                || estimate.latency < 0.0
                || estimate.latency > MAX_CAPTURE_LATENCY_SECONDS * 1000.0
                || (!multiTag && (!Double.isFinite(ambiguity) || ambiguity > MAX_SINGLE_TAG_AMBIGUITY))
                || !Double.isFinite(estimate.timestampSeconds)
                || estimate.timestampSeconds <= 0.0) {
            return Optional.empty();
        }

        return Optional.of(estimate);
    }

    private VisionEstimate toVisionEstimate(PoseEstimate estimate) {
        lastTimestampSeconds = estimate.timestampSeconds;

        int[] tagIds = getTagIds(estimate);
        boolean multiTag = tagIds.length >= 2;

        return new VisionEstimate(
            tagIds,
            new Pose3d(estimate.pose),
            Seconds.of(estimate.timestampSeconds),
            estimate.avgTagDist,
            multiTag ? 0.0 : avgAmbiguity(estimate),
            tagSpanMeters(tagIds),
            estimate.latency / 1000.0,
            multiTag || USE_VISION_ROTATION_FOR_SINGLE_TAG);
    }

    private int[] getTagIds(PoseEstimate estimate) {
        int[] tagIds = new int[estimate.rawFiducials.length];
        for (int i = 0; i < tagIds.length; i++) {
            tagIds[i] = estimate.rawFiducials[i].id;
        }
        return tagIds;
    }

    private double avgAmbiguity(PoseEstimate estimate) {
        double sum = 0.0;
        int count = 0;

        for (RawFiducial fiducial : estimate.rawFiducials) {
            if (Double.isFinite(fiducial.ambiguity) && fiducial.ambiguity >= 0.0) {
                sum += fiducial.ambiguity;
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
                    first.get().getTranslation().toTranslation2d().getDistance(second.get().getTranslation().toTranslation2d()));
            }
        }

        return maxSpan;
    }

    @Override
    protected void applyPipeline(CameraPipeline pipeline) {
        LimelightHelpers.setPipelineIndex(config.name, pipeline.index());
    }
}