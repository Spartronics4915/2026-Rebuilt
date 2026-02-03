package com.spartronics4915.frc2026.util;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseFusionEngine { 

    public ApriltagResult fusePoses(List<ApriltagResult> apriltagResults, VisionConfiguration config) {
        // Filter out results without poses
        List<ApriltagResult> validResults = apriltagResults.stream()
            .filter(ApriltagResult::hasPose)
            .toList();

        if (validResults.isEmpty()) return null;
        if (validResults.size() == 1) return validResults.get(0);
        if (!config.enablePoseFusion) return selectBestResult(validResults);
        if (validResults.size() < config.minCamerasForFusion) return selectBestResult(validResults);

        // Group by timestamp
        List<List<ApriltagResult>> groups = groupByTimestamp(validResults, config.fusionTimestampThreshold);
        List<ApriltagResult> largestGroup = groups.stream()
            .max((g1, g2) -> Integer.compare(g1.size(), g2.size()))
            .orElse(List.of());

        if (largestGroup.isEmpty()) return null;

        // Reject outliers
        List<ApriltagResult> filtered = rejectOutliers(largestGroup);
        
        if (filtered.isEmpty()) return null;

        return performWeightedFusion(filtered);
    }

    private List<List<ApriltagResult>> groupByTimestamp(
        List<ApriltagResult> results,
        double timestampThreshold
    ) {
        List<List<ApriltagResult>> groups = new ArrayList<>();
        List<ApriltagResult> sortedResults = new ArrayList<>(results);
        sortedResults.sort((r1, r2) -> Double.compare(r1.getTimestampSeconds(), r2.getTimestampSeconds()));

        List<ApriltagResult> currentGroup = new ArrayList<>();
        double groupTimestamp = -1;

        for (ApriltagResult result : sortedResults) {
            if (groupTimestamp < 0 || Math.abs(result.getTimestampSeconds() - groupTimestamp) <= timestampThreshold) {
                currentGroup.add(result);
                if (groupTimestamp < 0) groupTimestamp = result.getTimestampSeconds();
            } else {
                if (!currentGroup.isEmpty()) groups.add(new ArrayList<>(currentGroup));
                currentGroup.clear();
                currentGroup.add(result);
                groupTimestamp = result.getTimestampSeconds();
            }
        }

        if (!currentGroup.isEmpty()) groups.add(currentGroup);

        return groups;
    }

    private List<ApriltagResult> rejectOutliers(List<ApriltagResult> results) {
        if (results.size() <= 2) return results;

        Pose2d meanPose = calculateMeanPose(results);
        
        List<ApriltagResult> filtered = new ArrayList<>();
        double threshold = 5.0;

        for (ApriltagResult result : results) {
            double distance = calculateMahalanobisDistance(
                result.getEstimatedPose().get(), 
                meanPose, 
                result.getStdDevs()
            );
            if (distance < threshold) filtered.add(result);
        }

        return filtered.isEmpty() ? results : filtered;
    }

    private Pose2d calculateMeanPose(List<ApriltagResult> results) {
        double sumX = 0;
        double sumY = 0;
        double sumSin = 0;
        double sumCos = 0;

        for (ApriltagResult result : results) {
            Pose2d pose = result.getEstimatedPose().get();
            sumX += pose.getX();
            sumY += pose.getY();
            sumSin += Math.sin(pose.getRotation().getRadians());
            sumCos += Math.cos(pose.getRotation().getRadians());
        }

        int num = results.size();
        double meanTheta = Math.atan2(sumSin / num, sumCos / num);

        return new Pose2d(sumX / num, sumY / num, new Rotation2d(meanTheta));
    }

    private double calculateMahalanobisDistance(
        Pose2d pose1,
        Pose2d pose2,
        Matrix<N3, N1> stdDevs
    ) {
        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();
        double dtheta = Math.IEEEremainder(
            pose1.getRotation().getRadians() - pose2.getRotation().getRadians(), 
            2 * Math.PI
        );

        double distX = Math.abs(dx) / stdDevs.get(0, 0);
        double distY = Math.abs(dy) / stdDevs.get(1, 0);
        double distTheta = Math.abs(dtheta) / stdDevs.get(2, 0);

        return Math.sqrt(distX * distX + distY * distY + distTheta * distTheta);
    }

    private ApriltagResult performWeightedFusion(List<ApriltagResult> results) {
        double totalWeightX = 0;
        double totalWeightY = 0;
        double totalWeightTheta = 0;

        double weightedX = 0;
        double weightedY = 0;
        double weightedSin = 0;
        double weightedCos = 0;

        double avgTimestamp = results.stream()
            .mapToDouble(ApriltagResult::getTimestampSeconds)
            .average()
            .orElse(0);

        double avgLatency = results.stream()
            .mapToDouble(ApriltagResult::getLatencyMs)
            .average()
            .orElse(0);

        double avgDistance = results.stream()
            .mapToDouble(ApriltagResult::getAverageDistanceToTargets)
            .average()
            .orElse(0);

        double avgAmbiguity = results.stream()
            .mapToDouble(ApriltagResult::getAmbiguity)
            .average()
            .orElse(0);

        for (ApriltagResult result : results) {
            Matrix<N3, N1> stdDevs = result.getStdDevs();
            Pose2d pose = result.getEstimatedPose().get();
            
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
        double fusedTheta = Math.atan2(weightedSin / totalWeightTheta, weightedCos / totalWeightTheta);

        Pose2d fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));

        //double fusedStdDevX = Math.sqrt(1.0 / totalWeightX);
        //double fusedStdDevY = Math.sqrt(1.0 / totalWeightY);
        //double fusedStdDevTheta = Math.sqrt(1.0 / totalWeightTheta);

        double fusedStdDevX = 0.1;
        double fusedStdDevY = 0.1;
        double fusedStdDevTheta = 0.1;

        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(fusedStdDevX, fusedStdDevY, fusedStdDevTheta);

        String fusedCameraName = "fused[" + results.stream()
            .map(ApriltagResult::getCameraName)
            .reduce((s1, s2) -> s1 + "," + s2)
            .orElse("") + "]";

        // Combine all targets from fused results
        List<org.photonvision.targeting.PhotonTrackedTarget> allTargets = results.stream()
            .flatMap(r -> r.getTargets().stream())
            .toList();

        return new ApriltagResult.Builder()
            .cameraName(fusedCameraName)
            .timestamp(avgTimestamp)
            .latency(avgLatency)
            .pose(fusedPose)
            .stdDevs(fusedStdDevs)
            .targets(allTargets)
            .averageDistance(avgDistance)
            .ambiguity(avgAmbiguity)
            .build();
    }

    private ApriltagResult selectBestResult(List<ApriltagResult> results) {
        return results.stream()
            .min((r1, r2) -> {
                Matrix<N3, N1> std1 = r1.getStdDevs();
                Matrix<N3, N1> std2 = r2.getStdDevs();
                double sum1 = std1.get(0, 0) + std1.get(1, 0) + std1.get(2, 0);
                double sum2 = std2.get(0, 0) + std2.get(1, 0) + std2.get(2, 0);
                return Double.compare(sum1, sum2);
            }).orElse(null);
    }
}