package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class ApriltagResult implements ResultInterface {

    private final String sourceName;
    private final double timestampSeconds;
    private final double latencyMs;
    private final Pose2d pose;
    private Matrix<N3, N1> stdDevs;
    private final List<PhotonTrackedTarget> targets;
    private final int targetCount;
    private final double avgAmbiguity;
    private final double avgArea;

    public ApriltagResult(
        String name,
        double timestamp,
        double latency,
        Pose2d resultPose,
        Matrix<N3, N1> resultStdDevs,
        List<PhotonTrackedTarget> targets,
        double averageAmbiguity,
        double averageArea
    ) {
        this.sourceName = name;
        this.timestampSeconds = timestamp;
        this.latencyMs = latency;
        this.pose  = resultPose;
        this.stdDevs = resultStdDevs;
        this.targets = List.copyOf(targets);
        this.targetCount = targets.size();
        this.avgAmbiguity = averageAmbiguity;
        this.avgArea = averageArea;
    }

    //#region ------ Getters ------

    /**
     * Gets the name of the source
     */
    @Override
    public String getSourceName() {
        return sourceName;
    }

    /**
     * Gets the timestamp in seconds
     */
    @Override
    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    /**
     * Gets the latency in milliseconds
     */
    @Override
    public double getLatencyMs() {
        return latencyMs;
    }

    /**
     * Gets the pose
     */
    @Override
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Gets the standard deviations
     */
    @Override
    public Matrix<N3, N1> getStdDevs() {
        return stdDevs;
    }

    /**
     * Gets the list of targets
     */
    @Override
    public List<PhotonTrackedTarget> getTargets() {
        return targets;
    }

    /**
     * Gets the amount of targets
     */
    @Override
    public int getTargetCount() {
        return targetCount;
    }

    /**
     * Gets the average ambiguity
     */
    @Override
    public double getAmbiguity() {
        return avgAmbiguity;
    }

    /**
     * Gets the average area
     */
    @Override
    public double getAverageArea() {
        return avgArea;
    }

    //#endregion ------ Getters ------

    //#region ------ Setters ------

    public void setStdDevs(Matrix<N3, N1> newStdDevs) {
        stdDevs = newStdDevs;
    }

    //#endregion ------ Setters ------

}