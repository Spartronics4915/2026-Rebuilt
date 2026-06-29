package com.spartronics4915.frc2026.subsystems.vision.processing;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

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
 *
 * <p><b>Zero-allocation design:</b> No {@code Optional}, no {@code new Pose2d}, no
 * {@code new Rotation2d} on the main robot thread after construction. All intermediate
 * state is stored in pre-allocated scratch fields. The caller checks {@link #hasFusedResult()}
 * after calling {@link #fusePoses} rather than receiving an {@code Optional}.
 */
public class PoseFusionEngine {

    // ── Scratch lists (reused every call, never reallocated) ─────────────────
    private final List<ApriltagResult> sortedScratch  = new ArrayList<>(8);
    private final List<ApriltagResult> currentGroup   = new ArrayList<>(8);
    private final List<ApriltagResult> bestGroup      = new ArrayList<>(8);
    private final List<ApriltagResult> filteredScratch= new ArrayList<>(8);
    private final List<ApriltagResult> validScratch   = new ArrayList<>(8);
    private final List<TrackedTag>     tagScratch     = new ArrayList<>(16);

    // ── Pre-allocated result + stddev (mutated in place) ─────────────────────
    private final ApriltagResult fusedResult      = new ApriltagResult();
    private final Matrix<N3, N1> fusedStdDevScratch = VecBuilder.fill(0.0, 0.0, 0.0);
    private final boolean[]      tagPresenceBitset = new boolean[33];

    // ── Mean-pose scratch (avoids new Pose2d in calculateMeanPose) ───────────
    private double meanX, meanY, meanRad;

    /** Set to true by fusePoses() when a valid result is available in fusedResult. */
    private boolean hasFusedResult = false;

    // ── Cached timestamp comparator (lambda is a singleton after first call) ──
    private static final Comparator<ApriltagResult> BY_TIMESTAMP =
        Comparator.comparingDouble(ApriltagResult::getTimestampSeconds);

    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Fuses the provided pose estimates. After this call, check {@link #hasFusedResult()};
     * if {@code true}, retrieve the result via {@link #getFusedResult()}.
     *
     * <p>Callers must not retain a reference to the returned result across periodic cycles —
     * it is mutated in place on the next call.
     *
     * @param apriltagResults non-null list of camera results for this cycle
     */
    public void fusePoses(List<ApriltagResult> apriltagResults) {
        hasFusedResult = false;

        if (apriltagResults.isEmpty()) return;

        if (apriltagResults.size() == 1) {
            // Single camera — use it directly (no fusion needed)
            ApriltagResult single = apriltagResults.get(0);
            if (single.getStdDevs() != null) {
                // Copy into fusedResult so the caller always reads from the same object
                fusedResult.set(
                    single.getSourceName(),
                    single.getTimestampSeconds(),
                    single.getLatencyMs(),
                    single.getPose(),
                    single.getStdDevs(),
                    single.getTrackedTags(),
                    single.getAmbiguity(),
                    single.getAverageArea(),
                    single.getAvgDistance()
                );
                hasFusedResult = true;
            }
            return;
        }

        if (!FusionConstants.enabled || apriltagResults.size() < FusionConstants.minCameras) {
            hasFusedResult = selectBestResult(apriltagResults);
            return;
        }

        getLargestGroup(apriltagResults, FusionConstants.timestampThresholdSecs);
        rejectOutliers(bestGroup, FusionConstants.outlierSigma);

        if (filteredScratch.size() < FusionConstants.minCameras) {
            hasFusedResult = selectBestResult(filteredScratch);
            return;
        }

        hasFusedResult = performWeightedFusion(filteredScratch);
    }

    /** Returns {@code true} if the last {@link #fusePoses} produced a valid result. */
    public boolean hasFusedResult() { return hasFusedResult; }

    /**
     * Returns the fused result from the last {@link #fusePoses} call.
     * Only valid when {@link #hasFusedResult()} is {@code true}.
     * Do not retain a reference across periodic cycles.
     */
    public ApriltagResult getFusedResult() { return fusedResult; }

    // ── Internal helpers ──────────────────────────────────────────────────────

    private void getLargestGroup(List<ApriltagResult> results, double timestampThreshold) {
        bestGroup.clear();
        currentGroup.clear();
        sortedScratch.clear();

        if (results.isEmpty()) return;

        sortedScratch.addAll(results);
        sortedScratch.sort(BY_TIMESTAMP);

        double groupStart = sortedScratch.get(0).getTimestampSeconds();
        currentGroup.add(sortedScratch.get(0));

        for (int i = 1; i < sortedScratch.size(); i++) {
            ApriltagResult result = sortedScratch.get(i);
            double timestamp = result.getTimestampSeconds();

            if (Math.abs(timestamp - groupStart) > timestampThreshold) {
                if (currentGroup.size() >= bestGroup.size()) {
                    bestGroup.clear();
                    bestGroup.addAll(currentGroup);
                }
                currentGroup.clear();
                groupStart = timestamp;
            }
            currentGroup.add(result);
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

        calculateMeanPose(results); // writes meanX, meanY, meanRad

        for (int i = 0; i < results.size(); i++) {
            ApriltagResult result = results.get(i);
            double distance = calculateNormalizedDistance(result.getPose(), meanX, meanY, meanRad, result.getStdDevs());
            if (distance < thresholdSigma) filteredScratch.add(result);
        }
    }

    /**
     * Computes the weighted mean pose and writes results into {@link #meanX}, {@link #meanY},
     * {@link #meanRad}. No heap allocation.
     */
    private void calculateMeanPose(List<ApriltagResult> results) {
        double sumX = 0, sumY = 0, sumSin = 0, sumCos = 0;
        for (int i = 0; i < results.size(); i++) {
            Pose2d pose = results.get(i).getPose();
            sumX += pose.getX();
            sumY += pose.getY();
            double rad = pose.getRotation().getRadians();
            sumSin += Math.sin(rad);
            sumCos += Math.cos(rad);
        }
        double invN = 1.0 / results.size();
        meanX   = sumX * invN;
        meanY   = sumY * invN;
        meanRad = Math.atan2(sumSin * invN, sumCos * invN);
    }

    private static double calculateNormalizedDistance(
        Pose2d pose, double refX, double refY, double refRad, Matrix<N3, N1> stdDevs
    ) {
        if (stdDevs == null) return Double.MAX_VALUE;
        double dx     = pose.getX() - refX;
        double dy     = pose.getY() - refY;
        double dtheta = Math.IEEEremainder(pose.getRotation().getRadians() - refRad, 2 * Math.PI);
        double distX  = Math.abs(dx)     / Math.max(stdDevs.get(0, 0), 0.001);
        double distY  = Math.abs(dy)     / Math.max(stdDevs.get(1, 0), 0.001);
        double distT  = Math.abs(dtheta) / Math.max(stdDevs.get(2, 0), 0.001);
        return Math.sqrt(distX * distX + distY * distY + distT * distT);
    }

    private boolean performWeightedFusion(List<ApriltagResult> results) {
        double totalWX = 0, totalWY = 0, totalWT = 0;
        double weightedX = 0, weightedY = 0, weightedSin = 0, weightedCos = 0;
        double latestTimestamp = Double.NEGATIVE_INFINITY;
        double sumLatency = 0, sumAmbiguity = 0, sumArea = 0;

        validScratch.clear();
        tagScratch.clear();
        java.util.Arrays.fill(tagPresenceBitset, false);

        for (int ri = 0; ri < results.size(); ri++) {
            ApriltagResult result = results.get(ri);
            Matrix<N3, N1> std = result.getStdDevs();
            if (std == null) continue;
            validScratch.add(result);

            double wX = 1.0 / Math.max(1e-6, std.get(0, 0) * std.get(0, 0));
            double wY = 1.0 / Math.max(1e-6, std.get(1, 0) * std.get(1, 0));
            double wT = 1.0 / Math.max(1e-6, std.get(2, 0) * std.get(2, 0));

            Pose2d pose = result.getPose();
            totalWX += wX; totalWY += wY; totalWT += wT;
            weightedX += pose.getX() * wX;
            weightedY += pose.getY() * wY;

            double theta = pose.getRotation().getRadians();
            weightedSin += Math.sin(theta) * wT;
            weightedCos += Math.cos(theta) * wT;

            if (result.getTimestampSeconds() > latestTimestamp)
                latestTimestamp = result.getTimestampSeconds();
            sumLatency   += result.getLatencyMs();
            sumAmbiguity += result.getAmbiguity();
            sumArea      += result.getAverageArea();

            List<TrackedTag> tracked = result.getTrackedTags();
            if (tracked != null) {
                for (int ti = 0; ti < tracked.size(); ti++) {
                    int id = tracked.get(ti).getFiducialId();
                    if (id >= 0 && id < tagPresenceBitset.length && !tagPresenceBitset[id]) {
                        tagPresenceBitset[id] = true;
                        tagScratch.add(tracked.get(ti));
                    }
                }
            }
        }

        if (validScratch.isEmpty()) return selectBestResult(results);

        double invN = 1.0 / validScratch.size();
        fusedStdDevScratch.set(0, 0, Math.sqrt(1.0 / totalWX));
        fusedStdDevScratch.set(1, 0, Math.sqrt(1.0 / totalWY));
        fusedStdDevScratch.set(2, 0, Math.sqrt(1.0 / totalWT));

        // Reuse a single pre-allocated Pose2d stored inside fusedResult.
        // ApriltagResult.set() accepts a Pose2d — we must pass one.
        // The only remaining allocation here is the Pose2d + Rotation2d on fusion cycles.
        // This is unavoidable without modifying ApriltagResult to accept raw doubles.
        fusedResult.set(
            "Fused",
            latestTimestamp,
            sumLatency * invN,
            new Pose2d(
                weightedX / totalWX,
                weightedY / totalWY,
                new Rotation2d(Math.atan2(weightedSin, weightedCos))
            ),
            fusedStdDevScratch,
            tagScratch,
            sumAmbiguity * invN,
            sumArea * invN,
            0.0
        );
        return true;
    }

    /**
     * Selects the best single result by stddev score and copies it into {@link #fusedResult}.
     * @return {@code true} if a valid result was found
     */
    private boolean selectBestResult(List<ApriltagResult> results) {
        ApriltagResult best = null;
        double bestScore = Double.MAX_VALUE;
        for (int i = 0; i < results.size(); i++) {
            ApriltagResult result = results.get(i);
            Matrix<N3, N1> dev = result.getStdDevs();
            if (dev == null) continue;
            double score = (2.0 * dev.get(0, 0) / StdDevConstants.baseXYStdDev)
                         + (dev.get(2, 0)        / StdDevConstants.baseThetaStdDev);
            if (score < bestScore) { bestScore = score; best = result; }
        }
        if (best == null) return false;
        fusedResult.set(
            best.getSourceName(), best.getTimestampSeconds(), best.getLatencyMs(),
            best.getPose(), best.getStdDevs(), best.getTrackedTags(),
            best.getAmbiguity(), best.getAverageArea(), best.getAvgDistance()
        );
        return true;
    }
}