package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Computes pose estimation standard deviations from a quality score derived
 * from how large the tag(s) appear in the camera frame
 */
public class StdDevCalculator {

    /**
     * @param avgArea        Average tag area as fraction of camera frame (0–1).
     * @param numTags        Number of visible tags.
     * @param headingTrusted True when heading in this result is reliable, either a
     *                       PhotonVision multi-tag solve or a Limelight MegaTag2 result.
     */
    public static Matrix<N3, N1> calculate(double avgArea, int numTags, boolean headingTrusted) {
        double quality = computeQuality(avgArea, numTags);
        double scaleFactor = 1.0 / quality;

        double xyStd = baseXYDevs * scaleFactor;
        // Only trust vision heading when the result explicitly marks it as reliable.
        double thetaStd = headingTrusted ? baseThetaDevs * scaleFactor : largeVariance;

        return VecBuilder.fill(xyStd, xyStd, thetaStd);
    }

    /**
     * Computes a quality score in [MIN_QUALITY, 1.0].
     *
     * <p>Quality is driven primarily by tag area, a larger projected tag
     * means the camera is closer and the pixel-level solve is more accurate.
     * Multi-tag observations receive a bonus proportional to sqrt(numTags).
     */
    static double computeQuality(double avgArea, int numTags) {
        double areaQuality = Math.min(avgArea / saturationArea, 1.0);

        // Multi-tag bonus: sqrt(N) improvement, capped
        double tagBonus = Math.min(Math.sqrt(Math.max(numTags, 1)), maxTagBonus) / maxTagBonus;

        // Combined: area is dominant, tags provide a multiplier up to 2x
        double combined = areaQuality * (0.5 + 0.5 * tagBonus);

        return Math.max(combined, minQuality);
    }

}