package com.spartronics4915.frc2026.util.vision;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Calculates measurement standard deviations from observable AprilTag quality. */
public final class StdDevCalculator {
    private StdDevCalculator() {}

    public static Matrix<N3, N1> calculate(VisionEstimate estimate) {
        double distance = Math.max(
            estimate.avgTagDistanceMeters(),
            TAG_DISTANCE_REFERENCE_METERS * MIN_DISTANCE_FRACTION);
        double tagCount = Math.max(estimate.tagCount(), 1);

        // uncertainty ∝ distance² / independent tag count².
        double baseline = Math.pow(distance / TAG_DISTANCE_REFERENCE_METERS, 2.0)
            / Math.pow(tagCount, 2.0);

        // These are deliberately small, dimensionless quality penalties rather than a second
        // independent covariance model. Latency matters because it increases the chance that
        // the pose differs from the estimator state due to unmodeled motion.
        double ambiguityScale = estimate.tagCount() > 1
            ? 1.0 : 1.0 + AMBIGUITY_SCALE * MathUtil.clamp(estimate.avgTagAmbiguity(), 0.0, 1.0);

        double spreadScale = tagCount > 1
            ? MathUtil.clamp(
                Math.sqrt(
                    TAG_SPAN_REFERENCE_METERS / Math.max(TAG_SPAN_REFERENCE_METERS, estimate.tagSpanMeters())),
                    MIN_TAG_SPREAD_SCALE,
                    MAX_TAG_SPREAD_SCALE)
            : 1.0;

        double latencyScale = 1.0
            + LATENCY_SCALE * estimate.latencySeconds() / MAX_CAPTURE_LATENCY_SECONDS;

        double xyStdDev = XY_STD_DEV_COEFFICIENT
            * baseline
            * ambiguityScale
            * spreadScale
            * latencyScale;

        xyStdDev = MathUtil.clamp(xyStdDev, MIN_XY_STD_DEV_METERS, MAX_XY_STD_DEV_METERS);

        if (!estimate.useVisionRotation()) {
            return VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY);
        }

        double thetaStdDev = THETA_STD_DEV_COEFFICIENT
            * baseline
            * spreadScale
            * latencyScale;

        thetaStdDev = MathUtil.clamp(
            thetaStdDev,
            MIN_THETA_STD_DEV_RADIANS,
            MAX_THETA_STD_DEV_RADIANS);

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    public static boolean isValid(VisionEstimate estimate, double nowSeconds, double fieldLength, double fieldWidth) {
        if (estimate == null || estimate.tagCount() == 0) {
            return false;
        }

        double timestamp = estimate.timestamp().in(Seconds);
        double latency = estimate.latencySeconds();
        double x = estimate.pose().getX();
        double y = estimate.pose().getY();
        double z = estimate.pose().getZ();
        double distance = estimate.avgTagDistanceMeters();
        double ambiguity = estimate.avgTagAmbiguity();
        double span = estimate.tagSpanMeters();

        // Saftey guard to protect against wild numbers
        if (!Double.isFinite(timestamp)
                || !Double.isFinite(latency)
                || !Double.isFinite(x)
                || !Double.isFinite(y)
                || !Double.isFinite(z)
                || !Double.isFinite(distance)
                || !Double.isFinite(span)) {
            return false;
        }

        // Timestamp filtering, at a certain point results take to long to replay, so gating it needed
        if (timestamp <= 0.0
                || timestamp > nowSeconds + MAX_FUTURE_TIMESTAMP_SECONDS
                || latency < 0.0
                || latency > MAX_CAPTURE_LATENCY_SECONDS
                || distance > MAX_AVERAGE_TAG_DISTANCE_METERS) {
            return false;
        }

        // Translational sanity checks, our robot shouldnt be on the moon
        if (x < -FIELD_BOUNDARY_MARGIN_METERS
                || x > fieldLength + FIELD_BOUNDARY_MARGIN_METERS
                || y < -FIELD_BOUNDARY_MARGIN_METERS
                || y > fieldWidth + FIELD_BOUNDARY_MARGIN_METERS
                || z < MIN_ROBOT_Z_METERS
                || z > MAX_ROBOT_Z_METERS) {
            return false;
        }

        // Single-tag measurements should always carry a real ambiguity metric. MultiTag estimates
        // don't depend on the per-target ambiguity value, so they will report zero here.
        return estimate.tagCount() > 1 || Double.isFinite(ambiguity);
    }
}
