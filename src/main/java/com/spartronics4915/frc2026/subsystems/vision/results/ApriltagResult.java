package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import static com.spartronics4915.frc2026.Constants.VisionConstants.maxTagsPerFrame;

/**
 * Implementation of ResultInterface for AprilTag pipelines.
 */
public class ApriltagResult implements ResultInterface {

    private String sourceName = "";
    private double timestampSeconds = 0.0;
    private double latencyMs = 0.0;
    private Pose2d pose = new Pose2d();

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(0.0, 0.0, 0.0);

    /** Pre-allocated TrackedTag instances owned exclusively by this result. */
    private final TrackedTag[] ownedTags;

    private final List<TrackedTag> trackedTagsMutable = new ArrayList<>(maxTagsPerFrame);
    private final List<TrackedTag> trackedTagsView = Collections.unmodifiableList(trackedTagsMutable);

    private int targetCount = 0;
    private double avgAmbiguity = 0.0;
    private double avgArea = 0.0;

    /** No-arg constructor for pre-allocation in pools. */
    public ApriltagResult() {
        ownedTags = new TrackedTag[maxTagsPerFrame];
        for (int i = 0; i < maxTagsPerFrame; i++) {
            ownedTags[i] = new TrackedTag(0, 0.0, 0.0);
        }
    }

    /**
     * Updates all fields in this result.
     */
    public void set(
        String name,
        double timestamp,
        double latency,
        Pose2d resultPose,
        Matrix<N3, N1> resultStdDevs,
        List<TrackedTag> tags,
        double averageAmbiguity,
        double averageArea
    ) {
        this.sourceName = Objects.requireNonNullElse(name, "Unknown");
        this.timestampSeconds = timestamp;
        this.latencyMs = latency;
        this.pose = Objects.requireNonNullElse(resultPose, new Pose2d());

        if (resultStdDevs != null) {
            for (int i = 0; i < 3; i++) {
                this.stdDevs.set(i, 0, resultStdDevs.get(i, 0));
            }
        }

        this.trackedTagsMutable.clear();
        if (tags != null) {
            int count = Math.min(tags.size(), maxTagsPerFrame);
            for (int i = 0; i < count; i++) {
                TrackedTag src = tags.get(i);
                ownedTags[i].set(src.getFiducialId(), src.getArea(), src.getAmbiguity());
                this.trackedTagsMutable.add(ownedTags[i]);
            }
        }

        this.targetCount = this.trackedTagsMutable.size();
        this.avgAmbiguity = averageAmbiguity;
        this.avgArea = averageArea;
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
        return trackedTagsView;
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

}