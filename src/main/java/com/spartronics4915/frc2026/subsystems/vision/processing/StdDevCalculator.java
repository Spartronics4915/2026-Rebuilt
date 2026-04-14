package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Computes pose estimation standard deviations for a single camera.
 * Uses raw measurement quality factors without temporal smoothing.
 */
public class StdDevCalculator {

    public StdDevCalculator() {
        // Constructor is now empty as there is no state to maintain
    }

    /**
     * Calculates pose estimation standard deviations based on measurement quality factors.
     * Higher values indicate lower confidence.
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
        double latencySeconds = latencyMs / 1000.0;

        // Use raw numTags and avgArea directly instead of smoothed versions
        double ambiguityFactor = (numTags > 1) ? 1.0 : calculateAmbiguityFactor(avgAmbiguity);
        double areaFactor = calculateAreaFactor(avgArea);
        double tagCountFactor = calculateTagCountFactor(numTags);
        double latencyFactor = calculateLatencyFactor(latencySeconds);

        // Apply weights from Constants
        double xyMultiplier =
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;

        // Theta is less sensitive to area (0.5x) but more sensitive to latency (1.5x)
        double thetaMultiplier =
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight * 0.5) *
            Math.pow(latencyFactor, latencyWeight * 1.5) *
            tagCountFactor;

        return VecBuilder.fill(
            baseXYStdDev * xyMultiplier,
            baseXYStdDev * xyMultiplier,
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

    private static double calculateTagCountFactor(double tagCount) {
        return (1.0 / Math.log(Math.max(tagCount, 1.0) + 1.0)) * 1.4;
    }

}