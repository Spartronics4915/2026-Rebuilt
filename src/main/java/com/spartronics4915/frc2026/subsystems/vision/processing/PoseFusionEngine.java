package com.spartronics4915.frc2026.subsystems.vision.processing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
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

public class PoseFusionEngine {

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
     * Finds the largest group of results within the timestamp threshold.
     * On a size tie, the more recent group is preferred.
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
                // Use >= so that a later group of equal size replaces an earlier one,
                // ensuring we always work with the freshest data on a tie
                if (currentGroup.size() >= bestGroup.size()) {
                    bestGroup = new ArrayList<>(currentGroup);
                }
                currentGroup.clear();
                currentGroup.add(result);
                groupStart = ts;
            }
        }

        // Use >= here for the same reason as above
        if (currentGroup.size() >= bestGroup.size()) {
            bestGroup = currentGroup;
        }

        return bestGroup;
    }

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

    private static Pose2d calculateMeanPose(List<ApriltagResult> results) {
        double sumX = 0;
        double sumY = 0;
        double sumSin = 0;
        double sumCos = 0;

        for (ApriltagResult result : results) {
            Pose2d pose = result.getPose();
            sumX += pose.getX();
            sumY += pose.getY();
            sumSin += Math.sin(pose.getRotation().getRadians());
            sumCos += Math.cos(pose.getRotation().getRadians());
        }

        int num = results.size();
        double meanTheta = Math.atan2(sumSin / num, sumCos / num);

        return new Pose2d(sumX / num, sumY / num, new Rotation2d(meanTheta));
    }

    private static double calculateNormalizedDistance(
        Pose2d pose1,
        Pose2d pose2,
        Matrix<N3, N1> stdDevs
    ) {
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

        List<ApriltagResult> validResults = new ArrayList<>(results.size());

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

        double fusedY = weightedY / totalWeightY;
        double fusedX = weightedX / totalWeightX;
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

        List<PhotonTrackedTarget> allTargets = new ArrayList<>();
        for (ApriltagResult r : validResults) {
            outer:
                for (PhotonTrackedTarget t : r.getTargets()) {
                    int id = t.getFiducialId();
                    for (PhotonTrackedTarget existing : allTargets) {
                        if (existing.getFiducialId() == id) continue outer;
                    }
                    allTargets.add(t);
                }
        }

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

    private static Optional<ApriltagResult> selectBestResult(List<ApriltagResult> results) {
        ApriltagResult best = null;
        double bestScore = Double.MAX_VALUE;

        for (ApriltagResult r : results) {
            Matrix<N3, N1> std = r.getStdDevs();
            if (std == null) continue;

            double normalizedXY = std.get(0, 0) / StdDevConstants.baseXYStdDev;
            double normalizedTheta = std.get(2, 0) / StdDevConstants.baseThetaStdDev;

            double score = (2.0 * normalizedXY) + normalizedTheta;

            if (score < bestScore) {
                bestScore = score;
                best = r;
            }
        }

        return Optional.ofNullable(best);
    }

}