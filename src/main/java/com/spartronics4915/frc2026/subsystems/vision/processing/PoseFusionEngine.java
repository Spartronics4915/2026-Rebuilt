package com.spartronics4915.frc2026.subsystems.vision.processing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.photonvision.targeting.PhotonTrackedTarget;

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
     * @return Fused result, best single result, or null if no valid measurements
     */
    public static Optional<ApriltagResult> fusePoses(List<ApriltagResult> apriltagResults, VisionConfiguration config) {
        
        if (apriltagResults.size() == 1) {
            return Optional.of(apriltagResults.get(0));
        }
        
        if (!config.enablePoseFusion || apriltagResults.size() < config.minCamerasForFusion) {
            return selectBestResult(apriltagResults);
        }
        
        List<List<ApriltagResult>> timestampGroups = groupByTimestamp(
            apriltagResults, 
            config.fusionTimestampThreshold
        );
        
        List<ApriltagResult> largestGroup = timestampGroups.stream()
            .max(Comparator.comparingInt(List::size))
            .orElse(List.of());
        
        List<ApriltagResult> filtered = rejectOutliers(largestGroup, config.fusionOutlierThresholdSigma);
        
        if (filtered.size() < config.minCamerasForFusion) {
            return selectBestResult(filtered);
        }

        return performWeightedFusion(filtered);
    }
    
    /**
     * Group results by timestamp
     * 
     * @param results All results to group
     * @param timestampThreshold Maximum time difference (seconds) to be in same group
     * @return List of groups, each containing results from the same timestamp
     */
    private static List<List<ApriltagResult>> groupByTimestamp(
        List<ApriltagResult> results,
        double timestampThreshold
    ) {
        
        List<List<ApriltagResult>> groups = new ArrayList<>();

        List<ApriltagResult> sortedResults = new ArrayList<>(results);
        sortedResults.sort((r1, r2) -> Double.compare(r1.getTimestampSeconds(), r2.getTimestampSeconds()));
        
        List<ApriltagResult> currentGroup = new ArrayList<>();
        double groupTimestamp = -1;
        
        for (ApriltagResult result : sortedResults) {
            if (groupTimestamp < 0 || 
                Math.abs(result.getTimestampSeconds() - groupTimestamp) <= timestampThreshold) {
                currentGroup.add(result);
                if (groupTimestamp < 0) {
                    groupTimestamp = result.getTimestampSeconds();
                }
            } else {
                if (!currentGroup.isEmpty()) {
                    groups.add(new ArrayList<>(currentGroup));
                }
                currentGroup.clear();
                currentGroup.add(result);
                groupTimestamp = result.getTimestampSeconds();
            }
        }
        
        if (!currentGroup.isEmpty()) {
            groups.add(currentGroup);
        }
        
        return groups;
    }
    
    /**
     * Reject outlier measurements that significantly disagree with the group.
     * Uses Mahalanobis distance to account for uncertainty.
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
        
        List<ApriltagResult> filtered = new ArrayList<>();
        
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
     * @param pose1 The pose to measure from
     * @param pose2 The reference pose (typically the group mean)
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
        
        // [-π, π] 
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

        List<ApriltagResult> validResults = results.stream()
            .filter(r -> r.getStdDevs() != null)
            .toList();

        if (validResults.isEmpty()) return selectBestResult(results);
        
        double avgTimestamp = validResults.stream()
            .mapToDouble(ApriltagResult::getTimestampSeconds)
            .average()
            .orElse(0);
        
        double avgLatency = validResults.stream()
            .mapToDouble(ApriltagResult::getLatencyMs)
            .average()
            .orElse(0);
        
        double avgDistance = validResults.stream()
            .mapToDouble(ApriltagResult::getAverageDistanceToTargets)
            .average()
            .orElse(0);
        
        double avgAmbiguity = validResults.stream()
            .mapToDouble(ApriltagResult::getAmbiguity)
            .average()
            .orElse(0);
        
        double avgArea = validResults.stream()
            .mapToDouble(ApriltagResult::getAverageArea)
            .average()
            .orElse(0);
        
        double avgXAnisotropy = validResults.stream()
            .mapToDouble(ApriltagResult::getXAnisotropy)
            .average()
            .orElse(1.0);
        
        double avgYAnisotropy = validResults.stream()
            .mapToDouble(ApriltagResult::getYAnisotropy)
            .average()
            .orElse(1.0);
    
        ChassisSpeeds chassisSpeeds = validResults.stream()
            .map(ApriltagResult::getChassisSpeeds)
            .filter(speeds -> speeds != null)
            .findFirst()
            .orElse(new ChassisSpeeds());
        
        for (ApriltagResult result : validResults) {
            Matrix<N3, N1> stdDevs = result.getStdDevs();
            Pose2d pose = result.getPose();
            
            double weightX = 1.0 / (stdDevs.get(0, 0) * stdDevs.get(0, 0));
            double weightY = 1.0 / (stdDevs.get(1, 0) * stdDevs.get(1, 0));
            double weightTheta = 1.0 / (stdDevs.get(2, 0) * stdDevs.get(2, 0));
            
            totalWeightX += weightX;
            totalWeightY += weightY;
            totalWeightTheta += weightTheta;
            
            weightedX += pose.getX() * weightX;
            weightedY += pose.getY() * weightY;
            
            double theta = pose.getRotation().getRadians();
            weightedSin += Math.sin(theta) * weightTheta;
            weightedCos += Math.cos(theta) * weightTheta;
        }
        
        double fusedX = weightedX / totalWeightX;
        double fusedY = weightedY / totalWeightY;
        double fusedTheta = Math.atan2(
            weightedSin / totalWeightTheta,
            weightedCos / totalWeightTheta
        );
        
        Pose2d fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));
        
        double fusedStdDevX = Math.sqrt(1.0 / totalWeightX);
        double fusedStdDevY = Math.sqrt(1.0 / totalWeightY);
        double fusedStdDevTheta = Math.sqrt(1.0 / totalWeightTheta);
        
        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
            fusedStdDevX,
            fusedStdDevY,
            fusedStdDevTheta
        );
        

        String fusedCameraName = "fused[" +
            String.join(",", results.stream().map(ApriltagResult::getSourceName).toList()) + "]";
        
        List<PhotonTrackedTarget> allTargets = validResults.stream()
            .flatMap(r -> r.getTargets().stream())
            .collect(Collectors.toMap(
                PhotonTrackedTarget::getFiducialId,
                t -> t,
                (a, b) -> a
            ))
            .values()
            .stream()
            .toList();
        
        return Optional.of(
            new ApriltagResult(
                fusedCameraName,
                avgTimestamp,
                avgLatency,
                fusedPose,
                fusedStdDevs,
                allTargets,
                avgDistance,
                avgAmbiguity,
                avgArea,
                avgXAnisotropy,
                avgYAnisotropy,
                chassisSpeeds
            )
        );
    }
    
    /**
     * Select the single best result based on total uncertainty.
     * 
     * @param results Results to choose from
     * @return Result with lowest total uncertainty
     */
    private static Optional<ApriltagResult> selectBestResult(List<ApriltagResult> results) {
        return results.stream()
            .min(Comparator.comparingDouble(r -> {
                Matrix<N3, N1> std = r.getStdDevs();
                if (std == null) return Double.MAX_VALUE;
                return std.get(0, 0) + std.get(1, 0) + std.get(2, 0);
            }));
    }
    
}