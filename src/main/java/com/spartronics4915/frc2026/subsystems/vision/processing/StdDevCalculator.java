package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Computes pose estimation standard deviations for a single camera.
 */
public class StdDevCalculator {

    private final double alpha;

    private double smoothedArea;
    private double smoothedTagCount;

    private int previousNumTags;
    private boolean initialized;

    public StdDevCalculator(double alpha) {
        this.alpha = Math.max(0.0, Math.min(1.0, alpha));
        this.smoothedArea = 0.1;
        this.smoothedTagCount = 1;
        this.previousNumTags = 0;
        this.initialized = false;
    }

    /**
     * Calculates pose estimation standard deviations based on measurement quality factors.
     * Higher values indicate lower confidence. The result is used to weight this measurement
     * against odometry in the pose estimator.
     *
     * @param avgAmbiguity Average pose ambiguity, lower is better
     * @param avgArea Average tag area as fraction of frame, higher is better
     * @param latencyMs Pipeline latency in milliseconds
     * @param numTags Number of visible AprilTags, must be >= 1
     * @return 3x1 vector of [xStdDev, yStdDev, thetaStdDev] in meters and radians
     */
    public Matrix<N3, N1> calculate(
        double avgAmbiguity,
        double avgArea,
        double latencyMs,
        int numTags
    ) {
        // On first call or tag reacquisition, snap to current values instead of
        // bleeding in slowly from stale defaults.
        boolean reAcquired = (previousNumTags == 0 && numTags > 0);
        if (!initialized || reAcquired) {
            smoothedArea = avgArea;
            smoothedTagCount = numTags;
            initialized = true;
        } else {
            smoothedArea = alpha * avgArea + (1.0 - alpha) * smoothedArea;
            smoothedTagCount = alpha * numTags + (1.0 - alpha) * smoothedTagCount;
        }
        previousNumTags = numTags;

        double latencySeconds = latencyMs / 1000.0;

        double ambiguityFactor = (smoothedTagCount > 1) ? 1.0 : calculateAmbiguityFactor(avgAmbiguity);
        double areaFactor = calculateAreaFactor(smoothedArea);
        double tagCountFactor = calculateTagCountFactor(smoothedTagCount);
        double latencyFactor = calculateLatencyFactor(latencySeconds);

        double xyMultiplier =
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;

        // Theta is less sensitive to area (0.5x) but more sensitive to latency (1.5x)
        // since rotation errors compound more with stale measurements.
        double thetaMultiplier =
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight * 0.5) *
            Math.pow(latencyFactor, latencyWeight * 1.5) *
            tagCountFactor;

        return VecBuilder.fill(
            baseXYStdDev    * xyMultiplier,
            baseXYStdDev    * xyMultiplier,
            baseThetaStdDev * thetaMultiplier
        );
    }

    private static double calculateAmbiguityFactor(double ambiguity) {
        ambiguity = Math.max(0.0, Math.min(ambiguity, 0.25));
        double normalized = ambiguity / 0.1;
        return Math.min(1.0 + (normalized * normalized), 8.0);
    }

    private static double calculateAreaFactor(double area) {
        area = Math.max(0.001, Math.min(area, 1.0));
        return Math.min(0.1 / Math.sqrt(area), 5.0);
    }

    private static double calculateLatencyFactor(double latencySeconds) {
        return Math.min(1.0 + (latencySeconds * 5.0), 2.0);
    }

    private static double calculateTagCountFactor(double smoothedTagCount) {
        return (1.0 / Math.log(Math.max(smoothedTagCount, 1.0) + 1.0)) * 1.4;
    }

}