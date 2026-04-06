package com.spartronics4915.frc2026.subsystems.vision.processing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import com.spartronics4915.frc2026.Constants.VisionConstants.FusionConstants;
import com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Fuses pose estimates from multiple cameras into a single, more accurate measurement.
 * Optimized for zero-allocation performance during periodic execution.
 */
public class PoseFusionEngine {

    private final List<ApriltagResult> sortedScratch = new ArrayList<>(8);
    private final List<ApriltagResult> currentGroup = new ArrayList<>(8);
    private final List<ApriltagResult> bestGroup = new ArrayList<>(8);
    private final List<ApriltagResult> filteredScratch = new ArrayList<>(8);
    private final List<ApriltagResult> validScratch = new ArrayList<>(8);
    private final List<TrackedTag> tagScratch = new ArrayList<>(16);

    private final StringBuilder nameBuilder = new StringBuilder(64);

    private final ApriltagResult fusedResult = new ApriltagResult();
    private final Matrix<N3, N1> fusedStdDevScratch = VecBuilder.fill(0.0, 0.0, 0.0);
    private final boolean[] tagPresenceBitset = new boolean[33];

    /**
     * Fuses pose estimates from multiple cameras into a single measurement.
     */
    public Optional<ApriltagResult> fusePoses(List<ApriltagResult> apriltagResults) {
        if (apriltagResults.isEmpty()) {
            return Optional.empty();
        }

        if (apriltagResults.size() == 1) {
            return Optional.of(apriltagResults.get(0));
        }

        if (!FusionConstants.enabled || apriltagResults.size() < FusionConstants.minCameras) {
            return selectBestResult(apriltagResults);
        }

        getLargestTimestampGroup(apriltagResults, FusionConstants.timestampThresholdSecs);
        rejectOutliers(bestGroup, FusionConstants.outlierSigma);

        if (filteredScratch.size() < FusionConstants.minCameras) {
            return selectBestResult(filteredScratch);
        }

        return performWeightedFusion(filteredScratch);
    }

    private void getLargestTimestampGroup(List<ApriltagResult> results, double timestampThreshold) {
        sortedScratch.clear();
        sortedScratch.addAll(results);
        sortedScratch.sort(Comparator.comparingDouble(ApriltagResult::getTimestampSeconds));

        bestGroup.clear();
        currentGroup.clear();
        double groupStart = -1;

        for (int i = 0; i < sortedScratch.size(); i++) {
            ApriltagResult result = sortedScratch.get(i);
            double ts = result.getTimestampSeconds();

            if (groupStart < 0 || Math.abs(ts - groupStart) <= timestampThreshold) {
                if (groupStart < 0) groupStart = ts;
                currentGroup.add(result);
            } else {
                if (currentGroup.size() >= bestGroup.size()) {
                    bestGroup.clear();
                    bestGroup.addAll(currentGroup);
                }
                currentGroup.clear();
                currentGroup.add(result);
                groupStart = ts;
            }
        }

        if (currentGroup.size() >= bestGroup.size()) {
            bestGroup.clear();
            bestGroup.addAll(currentGroup);
        }
    }

    private void rejectOutliers(List<ApriltagResult> results, double thresholdSigma) {
        filteredScratch.clear();

        if (results.size() <= 2) {
            filteredScratch.addAll(results);
            return;
        }

        Pose2d meanPose = calculateMeanPose(results);

        for (int i = 0; i < results.size(); i++) {
            ApriltagResult result = results.get(i);
            double distance = calculateNormalizedDistance(result.getPose(), meanPose, result.getStdDevs());
            if (distance < thresholdSigma) filteredScratch.add(result);
        }

        if (filteredScratch.isEmpty()) filteredScratch.addAll(results);
    }

    private static Pose2d calculateMeanPose(List<ApriltagResult> results) {
        double sumX = 0;
        double sumY = 0;
        double sumSin = 0;
        double sumCos = 0;
        
        for (int i = 0; i < results.size(); i++) {
            Pose2d pose = results.get(i).getPose();
            sumX += pose.getX();
            sumY += pose.getY();
            double rad = pose.getRotation().getRadians();
            sumSin += Math.sin(rad);
            sumCos += Math.cos(rad);
        }

        double invN = 1.0 / results.size();
        return new Pose2d(
            sumX * invN, 
            sumY * invN,
            new Rotation2d(Math.atan2(sumSin * invN, sumCos * invN))
        );
    }

    private static double calculateNormalizedDistance(Pose2d pose1, Pose2d pose2, Matrix<N3, N1> stdDevs) {
        if (stdDevs == null) return Double.MAX_VALUE;

        double dx = pose1.getX() - pose2.getX();
        double dy = pose1.getY() - pose2.getY();
        double dtheta = Math.IEEEremainder(
            pose1.getRotation().getRadians() - pose2.getRotation().getRadians(), 
            2 * Math.PI
        );

        double distX = Math.abs(dx) / Math.max(stdDevs.get(0, 0), 0.001);
        double distY = Math.abs(dy) / Math.max(stdDevs.get(1, 0), 0.001);
        double distTheta = Math.abs(dtheta) / Math.max(stdDevs.get(2, 0), 0.001);

        return Math.sqrt(distX * distX + distY * distY + distTheta * distTheta);
    }

    private Optional<ApriltagResult> performWeightedFusion(List<ApriltagResult> results) {
        double totalWeightX = 0, totalWeightY = 0, totalWeightTheta = 0;
        double weightedX = 0, weightedY = 0, weightedSin = 0, weightedCos = 0;
        double latestTimestamp = Double.NEGATIVE_INFINITY;
        double sumLatency = 0, sumAmbiguity = 0, sumArea = 0;

        validScratch.clear();

        for (int i = 0; i < results.size(); i++) {
            ApriltagResult result = results.get(i);
            Matrix<N3, N1> stdDevs = result.getStdDevs();
            if (stdDevs == null) continue;
            validScratch.add(result);

            Pose2d pose = result.getPose();
            double wX = 1.0 / Math.max(1e-6, Math.pow(stdDevs.get(0, 0), 2));
            double wY = 1.0 / Math.max(1e-6, Math.pow(stdDevs.get(1, 0), 2));
            double wT = 1.0 / Math.max(1e-6, Math.pow(stdDevs.get(2, 0), 2));

            totalWeightX += wX; 
            totalWeightY += wY; 
            totalWeightTheta += wT;
            weightedX += pose.getX() * wX;
            weightedY += pose.getY() * wY;

            double theta = pose.getRotation().getRadians();
            weightedSin += Math.sin(theta) * wT;
            weightedCos += Math.cos(theta) * wT;

            if (result.getTimestampSeconds() > latestTimestamp) latestTimestamp = result.getTimestampSeconds();

            sumLatency += result.getLatencyMs();
            sumAmbiguity += result.getAmbiguity();
            sumArea += result.getAverageArea();
        }

        if (validScratch.isEmpty()) return selectBestResult(results);

        double invN = 1.0 / validScratch.size();

        fusedStdDevScratch.set(0, 0, Math.sqrt(1.0 / totalWeightX));
        fusedStdDevScratch.set(1, 0, Math.sqrt(1.0 / totalWeightY));
        fusedStdDevScratch.set(2, 0, Math.sqrt(1.0 / totalWeightTheta));

        Pose2d fusedPose = new Pose2d(
            weightedX / totalWeightX,
            weightedY / totalWeightY,
            new Rotation2d(Math.atan2(weightedSin / totalWeightTheta, weightedCos / totalWeightTheta))
        );

        tagScratch.clear();
        for (int i = 0; i < tagPresenceBitset.length; i++) tagPresenceBitset[i] = false;

        for (int i = 0; i < validScratch.size(); i++) {
            List<TrackedTag> tags = validScratch.get(i).getTrackedTags();
            for (int j = 0; j < tags.size(); j++) {
                TrackedTag t = tags.get(j);
                int id = t.getFiducialId();
                if (id >= 0 && id < tagPresenceBitset.length && !tagPresenceBitset[id]) {
                    tagPresenceBitset[id] = true;
                    tagScratch.add(t);
                }
            }
        }

        nameBuilder.setLength(0);
        nameBuilder.append("fused[");
        for (int i = 0; i < validScratch.size(); i++) {
            if (i > 0) nameBuilder.append(',');
            nameBuilder.append(validScratch.get(i).getSourceName());
        }
        nameBuilder.append(']');

        fusedResult.set(
            nameBuilder.toString(), 
            latestTimestamp,
            sumLatency * invN, 
            fusedPose, 
            fusedStdDevScratch,
            tagScratch, 
            sumAmbiguity * invN, 
            sumArea * invN
        );

        return Optional.of(fusedResult);
    }

    private static Optional<ApriltagResult> selectBestResult(List<ApriltagResult> results) {
        ApriltagResult best = null;
        double bestScore = Double.MAX_VALUE;
        for (int i = 0; i < results.size(); i++) {
            ApriltagResult result = results.get(i);
            Matrix<N3, N1> dev = result.getStdDevs();

            if (dev == null) continue;

            double score = (2.0 * dev.get(0, 0) / StdDevConstants.baseXYStdDev)
                + (dev.get(2, 0) / StdDevConstants.baseThetaStdDev);

            if (score < bestScore) {
                bestScore = score; 
                best = result; 
            }
        }
        return Optional.ofNullable(best);
    }

}