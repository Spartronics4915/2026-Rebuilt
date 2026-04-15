package com.spartronics4915.frc2026.subsystems.vision.samples;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Represents a robot pose estimate from vision with associated uncertainty and metadata. */
public class FieldPoseEstimate {

    private final Pose2d robotPoseMeters;
    private final double timestampSeconds;
    private final Matrix<N3, N1> measurementStdDevs;
    private final int numTags;

    /**
     * Creates a new field pose estimate.
     *
     * @param robotPoseMeters The estimated robot pose on the field in meters
     * @param timestampSeconds The timestamp when this estimate was captured
     * @param measurementStdDevs Standard deviations representing measurement uncertainty
     * @param numTags Number of AprilTags used in this pose estimate
     */
    public FieldPoseEstimate(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> measurementStdDevs,
        int numTags
    ) {
        this.robotPoseMeters = visionRobotPoseMeters;
        this.timestampSeconds = timestampSeconds;
        this.measurementStdDevs = measurementStdDevs;
        this.numTags = numTags;
    }

    public Pose2d getRobotPoseMeters() {
        return robotPoseMeters;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public Matrix<N3, N1> getMeasurementStdDevs() {
        return measurementStdDevs;
    }

    public int getNumTags() {
        return numTags;
    }
    
}