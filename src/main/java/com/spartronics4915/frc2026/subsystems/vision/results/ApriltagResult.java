package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class ApriltagResult implements ResultInterface {
    private final String cameraName;
    private final double timestampSeconds;
    private final double latencyMs;
    private final Optional<Pose2d> estimatedPose;
    private final Matrix<N3, N1> stdDevs;
    private final List<PhotonTrackedTarget> targets;
    private final int targetCount;
    private final double averageDistanceToTargets;
    private final double ambiguity;

    public ApriltagResult(
        String cameraName,
        double timestampSeconds,
        double latencyMs,
        Optional<Pose2d> estimatedPose,
        Matrix<N3, N1> stdDevs,
        List<PhotonTrackedTarget> targets,
        double averageDistanceToTargets,
        double ambiguity
    ) {
        this.cameraName = cameraName;
        this.timestampSeconds = timestampSeconds;
        this.latencyMs = latencyMs;
        this.estimatedPose = estimatedPose;
        this.stdDevs = stdDevs;
        this.targets = List.copyOf(targets);
        this.targetCount = targets.size();
        this.averageDistanceToTargets = averageDistanceToTargets;
        this.ambiguity = ambiguity;
    }

    @Override
    public String getCameraName() {
        return cameraName;
    }

    @Override
    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    @Override
    public double getLatencyMs() {
        return latencyMs;
    }

    public Optional<Pose2d> getEstimatedPose() {
        return estimatedPose;
    }

    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

    @Override
    public List<PhotonTrackedTarget> getTargets() {
        return targets;
    }

    @Override
    public int getTargetCount() {
        return targetCount;
    }

    @Override
    public double getAverageDistanceToTargets() {
        return averageDistanceToTargets;
    }

    @Override
    public double getAmbiguity() {
        return ambiguity;
    }

    @Override
    public boolean hasPose() {
        return estimatedPose.isPresent();
    }

    @Override
    public boolean hasValidData() {
        return hasPose();
    }

    public static class Builder {
        private String cameraName;
        private double timestampSeconds;
        private double latencyMs;
        private Optional<Pose2d> estimatedPose;
        private Matrix<N3, N1> stdDevs;
        private List<PhotonTrackedTarget> targets;
        private double averageDistanceToTargets;
        private double ambiguity;

        public Builder cameraName(String name) {
            this.cameraName = name;
            return this;
        }

        public Builder timestamp(double seconds) {
            this.timestampSeconds = seconds;
            return this;
        }

        public Builder latency(double ms) {
            this.latencyMs = ms;
            return this;
        }

        public Builder pose(Pose2d pose) {
            this.estimatedPose = Optional.ofNullable(pose);
            return this;
        }

        public Builder stdDevs(Matrix<N3, N1> stdDevs) {
            this.stdDevs = stdDevs;
            return this;
        }

        public Builder targets(List<PhotonTrackedTarget> targets) {
            this.targets = targets;
            return this;
        }

        public Builder averageDistance(double meters) {
            this.averageDistanceToTargets = meters;
            return this;
        }

        public Builder ambiguity(double ambiguity) {
            this.ambiguity = ambiguity;
            return this;
        }

        public ApriltagResult build() {
            return new ApriltagResult(
                cameraName,
                timestampSeconds,
                latencyMs,
                estimatedPose,
                stdDevs,
                targets,
                averageDistanceToTargets,
                ambiguity
            );
        }
    }
}