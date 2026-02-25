package com.spartronics4915.frc2026.subsystems.vision.processing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants;
import com.spartronics4915.frc2026.subsystems.vision.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Fuses pose estimates from multiple cameras into a single, more accurate measurement.
 */
public class PoseFusionEngine {

    /**
     * Fuse multiple AprilTag results into a single, more accurate pose estimate.
     *
     * @param apriltagResults List of results from different cameras
     * @param config Vision configuration with fusion settings
     * @return Fused result, best single result, or empty if no valid measurements
     */
    public static Optional<ApriltagResult> fusePoses(List<ApriltagResult> apriltagResults, VisionConfiguration config) {

        if (apriltagResults.size() == 1) {
            return Optional.of(apriltagResults.get(0));
        }

        if (!config.enablePoseFusion || apriltagResults.size() < config.minCamerasForFusion) {
            return selectBestResult(apriltagResults);
        }

        List<ApriltagResult> largestGroup = getLargestTimestampGroup(
            apriltagResults,
            config.fusionTimestampThreshold
        );

        List<ApriltagResult> filtered = rejectOutliers(largestGroup, config.fusionOutlierThresholdSigma);

        if (filtered.size() < config.minCamerasForFusion) {
            return selectBestResult(filtered);
        }

        return performWeightedFusion(filtered);
    }

    /**
     * Finds the largest group of results that fall within the timestamp threshold,
     * without building all groups first. Replaces the previous two-step
     * groupByTimestamp + stream().max() pattern.
     *
     * @param results All results to group
     * @param timestampThreshold Maximum time difference (seconds) to be in same group
     * @return The largest group of results sharing a close timestamp
     */
    private static List<ApriltagResult> getLargestTimestampGroup(
        List<ApriltagResult> results,
        double timestampThreshold
    ) {
        List<ApriltagResult> sorted = new ArrayList<>(results);
        sorted.sort(Comparator.comparingDouble(ApriltagResult::getTimestampSeconds));

        List<ApriltagResult> bestGroup = List.of();
        List<ApriltagResult> currentGroup = new ArrayList<>(sorted.size());
        double groupStart = -1;

        for (ApriltagResult result : sorted) {
            double ts = result.getTimestampSeconds();
            if (groupStart < 0 || Math.abs(ts - groupStart) <= timestampThreshold) {
                if (groupStart < 0) groupStart = ts;
                currentGroup.add(result);
            } else {
                if (currentGroup.size() > bestGroup.size()) {
                    bestGroup = new ArrayList<>(currentGroup);
                }
                currentGroup.clear();
                currentGroup.add(result);
                groupStart = ts;
            }
        }

        if (currentGroup.size() > bestGroup.size()) {
            bestGroup = currentGroup;
        }

        return bestGroup;
    }

    /**
     * Reject outlier measurements that significantly disagree with the group.
     * Uses a diagonal approximation of Mahalanobis distance to account for uncertainty.
     *
     * @param results Results to filter
     * @param thresholdSigma How many standard deviations away to reject (typically 2-3)
     * @return Filtered results with outliers removed
     */
    private static List<ApriltagResult> rejectOutliers(
        List<ApriltagResult> results,
        double thresholdSigma
    ) {
        if (results.size() <= 2) {
            return results;
        }

        Pose2d meanPose = calculateMeanPose(results);

        List<ApriltagResult> filtered = new ArrayList<>(results.size());

        for (ApriltagResult result : results) {
            double distance = calculateNormalizedDistance(
                result.getPose(),
                meanPose,
                result.getStdDevs()
            );

            if (distance < thresholdSigma) {
                filtered.add(result);
            }
        }

        return filtered.isEmpty() ? results : filtered;
    }

    /**
     * Calculates the unweighted mean of a list of poses. Rotation is averaged
     * using the circular mean to avoid wrap-around errors near ±π.
     *
     * @param results The poses to average — must not be empty
     * @return The mean pose
     */
    private static Pose2d calculateMeanPose(List<ApriltagResult> results) {
        double sumX = 0;
        double sumY = 0;
        double sumSin = 0;
        double sumCos = 0;

        for (ApriltagResult result : results) {
            Pose2d pose = result.getPose();
            sumX += pose.getX();
            sumY += pose.getY();

            // Circular mean for rotation
            sumSin += Math.sin(pose.getRotation().getRadians());
            sumCos += Math.cos(pose.getRotation().getRadians());
        }

        int num = results.size();
        double meanTheta = Math.atan2(sumSin / num, sumCos / num);

        return new Pose2d(sumX / num, sumY / num, new Rotation2d(meanTheta));
    }

    /**
     * Calculates the normalized distance between two poses, where each axis is
     * scaled by its corresponding standard deviation. This is a diagonal approximation
     * of true Mahalanobis distance — it does not account for cross-axis correlations,
     * but is sufficient for independent x/y/theta estimates.
     *
     * @param pose1   The pose to measure from
     * @param pose2   The reference pose (typically the group mean)
     * @param stdDevs Standard deviations for pose1's [x, y, theta] axes
     * @return Distance in units of standard deviations, or {@link Double#MAX_VALUE} if stdDevs is null
     */
    private static double calculateNormalizedDistance(
        Pose2d pose1,
        Pose2d pose2,
        Matrix<N3, N1> stdDevs
    ) {
        if (stdDevs == null) return Double.MAX_VALUE;

        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();

        // Wrap to [-π, π]
        double dtheta = Math.IEEEremainder(
            pose1.getRotation().getRadians() - pose2.getRotation().getRadians(),
            2 * Math.PI
        );

        double distX = Math.abs(dx) / Math.max(stdDevs.get(0, 0), 0.001);
        double distY = Math.abs(dy) / Math.max(stdDevs.get(1, 0), 0.001);
        double distTheta = Math.abs(dtheta) / Math.max(stdDevs.get(2, 0), 0.001);

        return Math.sqrt(distX * distX + distY * distY + distTheta * distTheta);
    }

    /**
     * Fuse multiple measurements using inverse variance weighting.
     * All aggregation is performed in a single loop to avoid multiple stream passes.
     *
     * @param results Filtered results to fuse
     * @return Single fused result
     */
    private static Optional<ApriltagResult> performWeightedFusion(List<ApriltagResult> results) {
        double totalWeightX = 0;
        double totalWeightY = 0;
        double totalWeightTheta = 0;
        double weightedX = 0;
        double weightedY = 0;
        double weightedSin = 0;
        double weightedCos = 0;

        double sumTimestamp = 0;
        double sumLatency = 0; 
        double sumDistance = 0;
        double sumAmbiguity = 0;
        double sumArea = 0;
        double sumXAnisotropy = 0; 
        double sumYAnisotropy = 0;

        ChassisSpeeds chassisSpeeds = null;

        // Reuse a sized list instead of streaming to .toList()
        List<ApriltagResult> validResults = new ArrayList<>(results.size());

        // Single pass: filter, accumulate weights, and sum all aggregate fields at once
        for (ApriltagResult result : results) {
            Matrix<N3, N1> stdDevs = result.getStdDevs();
            if (stdDevs == null) continue;

            validResults.add(result);

            Pose2d pose = result.getPose();
            double sx = stdDevs.get(0, 0);
            double sy = stdDevs.get(1, 0);
            double st = stdDevs.get(2, 0);

            double wX = 1.0 / (sx * sx);
            double wY = 1.0 / (sy * sy);
            double wT = 1.0 / (st * st);

            totalWeightX += wX;
            totalWeightY += wY;
            totalWeightTheta += wT;

            weightedX += pose.getX() * wX;
            weightedY += pose.getY() * wY;

            double theta = pose.getRotation().getRadians();
            weightedSin += Math.sin(theta) * wT;
            weightedCos += Math.cos(theta) * wT;

            sumTimestamp += result.getTimestampSeconds();
            sumLatency += result.getLatencyMs();
            sumDistance += result.getAverageDistanceToTargets();
            sumAmbiguity += result.getAmbiguity();
            sumArea += result.getAverageArea();
            sumXAnisotropy += result.getXAnisotropy();
            sumYAnisotropy += result.getYAnisotropy();

            if (chassisSpeeds == null) {
                chassisSpeeds = result.getChassisSpeeds();
            }
        }

        if (validResults.isEmpty()) return selectBestResult(results);

        double invN = 1.0 / validResults.size();

        // Compute fused pose
        double fusedX = weightedX / totalWeightX;
        double fusedY = weightedY / totalWeightY;
        double fusedTheta = Math.atan2(
            weightedSin / totalWeightTheta,
            weightedCos / totalWeightTheta
        );

        Pose2d fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));

        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
            Math.sqrt(1.0 / totalWeightX),
            Math.sqrt(1.0 / totalWeightY),
            Math.sqrt(1.0 / totalWeightTheta)
        );

        // Deduplicate targets using a plain loop + HashMap instead of stream + Collectors.toMap
        Map<Integer, PhotonTrackedTarget> targetMap = new HashMap<>();
        for (ApriltagResult r : validResults) {
            for (PhotonTrackedTarget t : r.getTargets()) {
                targetMap.putIfAbsent(t.getFiducialId(), t);
            }
        }
        List<PhotonTrackedTarget> allTargets = new ArrayList<>(targetMap.values());

        // Build camera name with StringBuilder instead of stream + String.join
        StringBuilder nameBuilder = new StringBuilder("fused[");
        for (int i = 0; i < validResults.size(); i++) {
            if (i > 0) nameBuilder.append(',');
            nameBuilder.append(validResults.get(i).getSourceName());
        }
        nameBuilder.append(']');

        return Optional.of(
            new ApriltagResult(
                nameBuilder.toString(),
                sumTimestamp * invN,
                sumLatency * invN,
                fusedPose,
                fusedStdDevs,
                allTargets,
                sumDistance * invN,
                sumAmbiguity * invN,
                sumArea * invN,
                sumXAnisotropy * invN,
                sumYAnisotropy * invN,
                chassisSpeeds != null ? chassisSpeeds : new ChassisSpeeds()
            )
        );
    }

    /**
     * Select the single best result based on total normalized uncertainty.
     * Each axis is divided by its base std dev before summing so that
     * translational (meters) and rotational (radians) uncertainty are
     * comparable rather than added as raw mixed units.
     *
     * @param results Results to choose from
     * @return Result with lowest total normalized uncertainty
     */
    private static Optional<ApriltagResult> selectBestResult(List<ApriltagResult> results) {
        return results.stream()
            .min(Comparator.comparingDouble(r -> {
                Matrix<N3, N1> std = r.getStdDevs();
                if (std == null) return Double.MAX_VALUE;

                double normalizedXY    = std.get(0, 0) / StdDevConstants.baseXYStdDev;
                double normalizedTheta = std.get(2, 0) / StdDevConstants.baseThetaStdDev;

                // Weight xy twice since it has two independent axes (x and y)
                return (2.0 * normalizedXY) + normalizedTheta;
            }));
    }

}