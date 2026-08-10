package com.spartronics4915.frc2026.subsystems.vision.processing;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Combines simultaneous camera estimates using their reported measurement uncertainty. */
public class PoseFusionEngine {
    private final ApriltagResult fusedResult = new ApriltagResult();
    private final Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
    private final List<TrackedTag> tagScratch = new ArrayList<>();

    public Optional<ApriltagResult> fusePoses(List<ApriltagResult> results) {
        if (results.isEmpty()) return Optional.empty();
        if (results.size() == 1) return Optional.of(results.get(0));

        double weightX = 0.0;
        double weightY = 0.0;
        double weightTheta = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedSin = 0.0;
        double weightedCos = 0.0;
        double latestTimestamp = Double.NEGATIVE_INFINITY;
        double averageLatency = 0.0;
        double averageAmbiguity = 0.0;
        double averageArea = 0.0;

        tagScratch.clear();
        for (ApriltagResult result : results) {
            Matrix<N3, N1> stdDevs = result.getStdDevs();
            if (stdDevs == null) continue;

            double xWeight = inverseVariance(stdDevs.get(0, 0));
            double yWeight = inverseVariance(stdDevs.get(1, 0));
            double thetaWeight = inverseVariance(stdDevs.get(2, 0));
            Pose2d pose = result.getPose();

            weightX += xWeight;
            weightY += yWeight;
            weightTheta += thetaWeight;
            weightedX += pose.getX() * xWeight;
            weightedY += pose.getY() * yWeight;
            weightedSin += Math.sin(pose.getRotation().getRadians()) * thetaWeight;
            weightedCos += Math.cos(pose.getRotation().getRadians()) * thetaWeight;
            latestTimestamp = Math.max(latestTimestamp, result.getTimestampSeconds());
            averageLatency += result.getLatencyMs();
            averageAmbiguity += result.getAmbiguity();
            averageArea += result.getAverageArea();
            tagScratch.addAll(result.getTrackedTags());
        }

        if (weightX == 0.0 || weightY == 0.0 || weightTheta == 0.0) return Optional.empty();

        fusedStdDevs.set(0, 0, Math.sqrt(1.0 / weightX));
        fusedStdDevs.set(1, 0, Math.sqrt(1.0 / weightY));
        fusedStdDevs.set(2, 0, Math.sqrt(1.0 / weightTheta));
        double count = results.size();
        fusedResult.set(
            "Fused",
            latestTimestamp,
            averageLatency / count,
            new Pose2d(
                weightedX / weightX,
                weightedY / weightY,
                new Rotation2d(Math.atan2(weightedSin, weightedCos))
            ),
            fusedStdDevs,
            tagScratch,
            averageAmbiguity / count,
            averageArea / count,
            0.0
        );
        return Optional.of(fusedResult);
    }

    private static double inverseVariance(double standardDeviation) {
        return 1.0 / Math.max(1e-6, standardDeviation * standardDeviation);
    }
}
