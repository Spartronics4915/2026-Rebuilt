package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Immutable result produced by any AprilTag-capable processor.
 */
public class ApriltagResult implements ResultInterface {

    private final String sourceName;
    private final double timestampSeconds;
    private final double latencyMs;
    private final Pose2d pose;
    private Matrix<N3, N1> stdDevs;
    private final List<TrackedTag> trackedTags;
    private final int targetCount;
    private final double avgAmbiguity;
    private final double avgArea;

    private final Optional<Transform3d> singleTagCameraToTarget;

    private final boolean headingTrusted;

    public ApriltagResult(
        String name,
        double timestamp,
        double latency,
        Pose2d resultPose,
        Matrix<N3, N1> resultStdDevs,
        List<TrackedTag> tags,
        double averageAmbiguity,
        double averageArea,
        Optional<Transform3d> singleTagCameraToTarget,
        boolean headingTrusted
    ) {
        this.sourceName = name;
        this.timestampSeconds = timestamp;
        this.latencyMs = latency;
        this.pose = resultPose;
        this.stdDevs = resultStdDevs;
        this.trackedTags = List.copyOf(tags);
        this.targetCount = tags.size();
        this.avgAmbiguity = averageAmbiguity;
        this.avgArea = averageArea;
        this.singleTagCameraToTarget = singleTagCameraToTarget;
        this.headingTrusted = headingTrusted;
    }

    public ApriltagResult(
        String name,
        double timestamp,
        double latency,
        Pose2d resultPose,
        Matrix<N3, N1> resultStdDevs,
        List<TrackedTag> tags,
        double averageAmbiguity,
        double averageArea
    ) {
        this(
            name, 
            timestamp, 
            latency, 
            resultPose, 
            resultStdDevs, 
            tags,
            averageAmbiguity, 
            averageArea, 
            Optional.empty(), 
            false
        );
    }

    @Override public String getSourceName() {
        return sourceName; 
    }

    @Override public double getTimestampSeconds() { 
        return timestampSeconds; 
    }

    @Override public double getLatencyMs() {
        return latencyMs; 
    }

    @Override public Pose2d getPose() { 
        return pose; 
    }

    @Override public Matrix<N3, N1> getStdDevs() { 
        return stdDevs; 
    }

    @Override public List<TrackedTag> getTrackedTags() { 
        return trackedTags; 
    }

    @Override public int getTargetCount() { 
        return targetCount; 
    }

    @Override public double getAmbiguity() { 
        return avgAmbiguity; 
    }

    @Override public double getAverageArea() { 
        return avgArea; 
    }

    public Optional<Transform3d> getSingleTagCameraToTarget() {
        return singleTagCameraToTarget;
    }

    public boolean isMultiTag() {
        return targetCount > 1;
    }

    public boolean isHeadingTrusted() {
        return headingTrusted;
    }

    public void setStdDevs(Matrix<N3, N1> newStdDevs) {
        stdDevs = newStdDevs;
    }

}