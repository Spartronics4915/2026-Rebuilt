package com.spartronics4915.frc2026.util;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.strategies.PipelineStrategyInterface.PoseEstimate;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseFusionEngine {

    public PoseEstimate fusePoses(List<PoseEstimate> poseEstimates, VisionConfiguration config) {
        if (poseEstimates.isEmpty()) return null;
        if (poseEstimates.size() == 1) return poseEstimates.get(0);
        if (!config.enablePoseFusion) return selectBestPose(poseEstimates);
        if (poseEstimates.size() < config.minCamerasForFusion) return selectBestPose(poseEstimates);

        List<List<PoseEstimate>> groups = groupByTimestamp(poseEstimates, config.fusionTimestampThreshold);
        List<PoseEstimate> largestGroup = groups.stream()
            .max((g1, g2) -> Integer.compare(g1.size(), g2.size()))
            .orElse(List.of());

        if (largestGroup.isEmpty()) return null;

        List<PoseEstimate> filtered = rejectOutliers(largestGroup);
        
        if (filtered.isEmpty()) return null;

        return performWeightedFusion(filtered);
    }

    private List<List<PoseEstimate>> groupByTimestamp(
        List<PoseEstimate> poses,
        double timestampThreshold
    ) {
        
        List<List<PoseEstimate>> groups = new ArrayList<>();
        List<PoseEstimate> sortedPoses = new ArrayList<>(poses);
        sortedPoses.sort((p1, p2) -> Double.compare(p1.getTimestamp(), p2.getTimestamp()));

        List<PoseEstimate> currentGroup = new ArrayList<>();
        double groupTimestamp = -1;

        for (PoseEstimate pose : sortedPoses) {
            if (groupTimestamp < 0 || Math.abs(pose.getTimestamp() - groupTimestamp) <= timestampThreshold) {
                currentGroup.add(pose);
                if (groupTimestamp < 0) groupTimestamp = pose.getTimestamp();
            } else {
                if (!currentGroup.isEmpty()) groups.add(new ArrayList<>(currentGroup));
                currentGroup.clear();
                currentGroup.add(pose);
                groupTimestamp = pose.getTimestamp();
            }
        }

        if (!currentGroup.isEmpty()) groups.add(currentGroup);

        return groups;
    }

    private List<PoseEstimate> rejectOutliers(List<PoseEstimate> poses) {
        if (poses.size() <= 2) return poses;

        Pose2d meanPose = calculateMeanPose(poses);
        
        List<PoseEstimate> filtered = new ArrayList<>();
        double threshold = 3.0;

        for (PoseEstimate pose : poses) {
            double distance = calculateMahalanobisDistance(pose.getPose(), meanPose, pose.getStdDevs());
            if (distance < threshold) filtered.add(pose);
        }

        return filtered.isEmpty() ? poses : filtered;
    }

    private Pose2d calculateMeanPose(List<PoseEstimate> poses) {
        double sumX = 0;
        double sumY = 0;
        double sumSin = 0;
        double sumCos = 0;

        for (PoseEstimate pose : poses) {
            sumX += pose.getPose().getX();
            sumY += pose.getPose().getY();
            sumSin += Math.sin(pose.getPose().getRotation().getRadians());
            sumCos += Math.cos(pose.getPose().getRotation().getRadians());
        }

        int num = poses.size();
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
        double dtheta = pose1.getRotation().getRadians() - pose2.getRotation().getRadians();

        dtheta = Math.IEEEremainder(pose1.getRotation().getRadians() - pose2.getRotation().getRadians(), 2 * Math.PI);

        double distX = Math.abs(dx) / stdDevs.get(0, 0);
        double distY = Math.abs(dy) / stdDevs.get(1, 0);
        double distTheta = Math.abs(dtheta) / stdDevs.get(2, 0);

        return Math.sqrt(distX * distX + distY * distY + distTheta * distTheta);
    }

    private PoseEstimate performWeightedFusion(List<PoseEstimate> poses) {
        double totalWeightX = 0;
        double totalWeightY = 0;
        double totalWeightTheta = 0;

        double weightedX = 0;
        double weightedY = 0;
        double weightedSin = 0;
        double weightedCos = 0;

        double avgTimestamp = poses.stream()
            .mapToDouble(PoseEstimate::getTimestamp)
            .average()
            .orElse(0);

        for (PoseEstimate pose : poses) {
            Matrix<N3, N1> stdDevs = pose.getStdDevs();
            
            double weightX = 1.0 / (stdDevs.get(0, 0) * stdDevs.get(0, 0));
            double weightY = 1.0 / (stdDevs.get(1, 0) * stdDevs.get(1, 0));
            double weightTheta = 1.0 / (stdDevs.get(2, 0) * stdDevs.get(2, 0));

            totalWeightX += weightX;
            totalWeightY += weightY;
            totalWeightTheta += weightTheta;

            weightedX += pose.getPose().getX() * weightX;
            weightedY += pose.getPose().getY() * weightY;
            
            double theta = pose.getPose().getRotation().getRadians();
            weightedSin += Math.sin(theta) * weightTheta;
            weightedCos += Math.cos(theta) * weightTheta;
        }

        double fusedX = weightedX / totalWeightX;
        double fusedY = weightedY / totalWeightY;
        double fusedTheta = Math.atan2(weightedSin / totalWeightTheta, weightedCos / totalWeightTheta);

        Pose2d fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));

        double fusedStdDevX = Math.sqrt(1.0 / totalWeightX);
        double fusedStdDevY = Math.sqrt(1.0 / totalWeightY);
        double fusedStdDevTheta = Math.sqrt(1.0 / totalWeightTheta);

        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(fusedStdDevX, fusedStdDevY, fusedStdDevTheta);

        String source = "fused[" + poses.stream()
            .map(PoseEstimate::getSource)
            .reduce((s1, s2) -> s1 + "," + s2)
            .orElse("") + "]";

        return new PoseEstimate(fusedPose, avgTimestamp, fusedStdDevs, source);
    }

    private PoseEstimate selectBestPose(List<PoseEstimate> poses) {
        return poses.stream()
            .min((p1, p2) -> {
                double std1 = p1.getStdDevs().get(0, 0) + p1.getStdDevs().get(1, 0) + p1.getStdDevs().get(2, 0);
                double std2 = p2.getStdDevs().get(0, 0) + p2.getStdDevs().get(1, 0) + p2.getStdDevs().get(2, 0);
                return Double.compare(std1, std2);
            }).orElse(null);
    }
}
