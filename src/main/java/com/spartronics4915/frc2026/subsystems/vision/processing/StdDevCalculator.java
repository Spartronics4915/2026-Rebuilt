package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StdDevCalculator {

    private double smoothedDistance;
    private double smoothedArea;
    private double smoothedTagCount;
    private int previousNumTags;
    private boolean initialized;

    public StdDevCalculator() {
        smoothedDistance = 2.0;
        smoothedArea = 0.1;
        smoothedTagCount = 1;
        previousNumTags = 0;
        initialized = false;
    }

    /**
     * Calculates pose estimation standard deviations based on measurement quality factors.
     * Higher values indicate lower confidence. The result is used to weight this measurement
     * against odometry in the pose estimator.
     *
     * <p>Distance and area inputs are smoothed via an exponential moving average to reduce
     * frame-to-frame jitter. On first call or tag re-acquisition, values snap immediately
     * rather than bleeding in from defaults.
     *
     * @param avgDistance Average distance to visible tags in meters
     * @param avgAmbiguity Average pose ambiguity [0.0, 0.25], lower is better
     * @param avgArea Average tag area as fraction of frame [0.0, 1.0], higher is better
     * @param xAnisotropy Horizontal view angle uncertainty multiplier [1.0, 10.0]
     * @param yAnisotropy Vertical view angle uncertainty multiplier [1.0, 10.0]
     * @param chassisSpeeds Current robot velocity
     * @param latencyMs Pipeline latency in milliseconds
     * @param numTags Number of visible AprilTags; must be >= 1
     * @return 3x1 vector of [xStdDev, yStdDev, thetaStdDev] in meters and radians
     */
    public Matrix<N3, N1> calculate(
        double avgDistance,
        double avgAmbiguity,
        double avgArea,
        double xAnisotropy,
        double yAnisotropy,
        ChassisSpeeds chassisSpeeds,
        double latencyMs,
        int numTags
    ) {
        // On first call or tag reacquisition, snap to current values instead of
        // bleeding in slowly from stale defaults
        boolean reAcquired = (previousNumTags == 0 && numTags > 0);
        if (!initialized || reAcquired) {
            smoothedDistance = avgDistance;
            smoothedArea = avgArea;
            smoothedTagCount = numTags;
            initialized = true;
        } else {
            smoothedDistance = smoothingAlpha * avgDistance + (1.0 - smoothingAlpha) * smoothedDistance;
            smoothedArea = smoothingAlpha * avgArea + (1.0 - smoothingAlpha) * smoothedArea;
            smoothedTagCount = smoothingAlpha * numTags + (1.0 - smoothingAlpha) * smoothedTagCount;
        }
        previousNumTags = numTags;

        double latencySeconds = latencyMs / 1000.0;

        double distanceFactor = calculateDistanceFactor(smoothedDistance);
        double ambiguityFactor = (smoothedTagCount > 1) ? 1.0 : calculateAmbiguityFactor(avgAmbiguity);
        double areaFactor = calculateAreaFactor(smoothedArea);
        double anisotropyFactor = calculateAnisotropyFactor(xAnisotropy, yAnisotropy);
        double latencyFactor = calculateLatencyFactor(latencySeconds);
        double tagCountFactor = calculateTagCountFactor(smoothedTagCount);
        double motionFactor = calculateMotionFactor(chassisSpeeds);

        double xyMultiplier =
            Math.pow(distanceFactor, distanceWeight) *
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight) *
            Math.pow(anisotropyFactor, anisotropyWeight) *
            Math.pow(motionFactor, motionWeight) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;

        /*
         * Theta is less sensitive to distance (0.7x) and tag area (0.5x)
         * but more sensitive to anisotropy (1.3x) and motion blur (1.5x)
         * since rotation errors compound with both viewing angle and motion
         */
        double thetaMultiplier =
            Math.pow(distanceFactor, distanceWeight * 0.7) *
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight * 0.5) *
            Math.pow(anisotropyFactor, anisotropyWeight * 1.3) *
            Math.pow(motionFactor, motionWeight * 1.5) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;

        double xyStdDev = baseXYStdDev * xyMultiplier;
        double thetaStdDev = baseThetaStdDev * thetaMultiplier;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    private static double calculateDistanceFactor(double distance) {
        double normalized = Math.max(distance, 0.1) / 2.0;
        double factor = normalized * normalized;
        return Math.min(factor, 10.0);
    }

    private static double calculateAmbiguityFactor(double ambiguity) {
        ambiguity = Math.max(0.0, Math.min(ambiguity, 0.25));
        double normalized = ambiguity / 0.1;
        double factor = 1.0 + (normalized * normalized);
        return Math.min(factor, 8.0);
    }

    private static double calculateAreaFactor(double area) {
        area = Math.max(0.001, Math.min(area, 1.0));
        return Math.min(0.1 / Math.sqrt(area), 5.0);
    }

    private static double calculateAnisotropyFactor(double xAnisotropy, double yAnisotropy) {
        xAnisotropy = Math.max(1.0, Math.min(xAnisotropy, 10.0));
        yAnisotropy = Math.max(1.0, Math.min(yAnisotropy, 10.0));
        double combined = Math.sqrt(xAnisotropy * yAnisotropy);
        return Math.min(combined, 4.0);
    }

    private static double calculateMotionFactor(ChassisSpeeds speeds) {
        double totalMotion = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            + Math.abs(speeds.omegaRadiansPerSecond);
        double factor = 1.0 + (totalMotion * 0.3);
        return Math.min(factor, 3.0);
    }

    private static double calculateLatencyFactor(double latencySeconds) {
        double factor = 1.0 + (latencySeconds * 5.0);
        return Math.min(factor, 2.0);
    }

    private static double calculateTagCountFactor(double smoothedTagCount) {
        return 1.0 / Math.sqrt(smoothedTagCount + 0.5);
    }
}