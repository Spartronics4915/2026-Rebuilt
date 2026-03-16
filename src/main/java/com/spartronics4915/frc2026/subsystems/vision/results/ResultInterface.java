package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Common interface for all vision pipeline results, regardless of the underlying
 * camera system (PhotonVision, Limelight, etc.).
 */
public interface ResultInterface {

    //#region ------ Getters ------

    /** Human-readable name of the camera or fused source that produced this result. */
    String getSourceName();

    /** Capture timestamp in seconds (FPGA time, latency-adjusted). */
    double getTimestampSeconds();

    /** End-to-end pipeline latency in milliseconds. */
    double getLatencyMs();

    /** Robot pose estimated from this result, in field coordinates. */
    Pose2d getPose();

    /**
     * 3x1 standard-deviation vector [xStdDev, yStdDev, thetaStdDev]
     */
    Matrix<N3, N1> getStdDevs();

    /**
     * The individual AprilTags that contributed to this pose estimate
     */
    List<TrackedTag> getTrackedTags();

    /** Number of AprilTags that contributed to this estimate. */
    int getTargetCount();

    /**
     * Average pose ambiguity across all contributing tags (0-1, lower is better).
     * For multi-tag and MegaTag2 results this is typically close to 0.
     */
    double getAmbiguity();

    /** Average tag area as a fraction of the camera frame (0-1). */
    double getAverageArea();

    //#endregion
    
}