package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Fuses accepted pose estimates from multiple cameras into a single measurement.
 */
public class PoseFusionEngine {

    // Supplier of historical robot pose keyed by timestamp (from SwerveSubsystem).
    private final java.util.function.Function<Double, Optional<Pose2d>> poseBufferSupplier;

    private final List<TrackedTag> tagScratch = new ArrayList<>(16);
    private final StringBuilder nameBuilder = new StringBuilder(64);

    public PoseFusionEngine(
        java.util.function.Function<Double, Optional<Pose2d>> poseBufferSupplier
    ) {
        this.poseBufferSupplier = poseBufferSupplier;
    }

    /**
     * Fuses a list of accepted results (one per camera) into a single estimate.
     *
     * @param accepted Non-empty list of accepted per-camera results.
     * @return Fused result, or the best single result if fusion is not possible.
     */
    public Optional<ApriltagResult> fuse(List<ApriltagResult> accepted) {
        if (accepted.isEmpty()) return Optional.empty();
        if (accepted.size() == 1) return Optional.of(accepted.get(0));

        // Sort descending by quality (tighter stdDevs = better) so iterative
        // fusion always starts with the most trusted pair.
        accepted.sort((a, b) -> {
            double qa = a.getStdDevs() == null ? Double.MAX_VALUE : a.getStdDevs().get(0, 0);
            double qb = b.getStdDevs() == null ? Double.MAX_VALUE : b.getStdDevs().get(0, 0);
            return Double.compare(qa, qb); // ascending, tighter first
        });

        ApriltagResult running = accepted.get(0);
        for (int i = 1; i < accepted.size(); i++) {
            Optional<ApriltagResult> merged = fuseTwo(running, accepted.get(i));
            if (merged.isPresent()) running = merged.get();
        }
        return Optional.of(running);
    }


    /**
     * Fuses exactly two results using inverse-variance weighting.
     *
     * <p>The older result is previewed forward to the newer timestamp via the
     * odometry buffer before fusion, so both represent the robot's pose at the
     * same moment
     */
    private Optional<ApriltagResult> fuseTwo(ApriltagResult a, ApriltagResult b) {
        if (a.getStdDevs() == null || b.getStdDevs() == null) {
            return a.getStdDevs() != null ? Optional.of(a) : Optional.of(b);
        }

        // Ensure b is always the newer measurement.
        if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
            ApriltagResult tmp = a; a = b; b = tmp;
        }

        // Preview a's pose forward to b's timestamp using odometry interpolation.
        Pose2d previewedPoseA = previewToTimestamp(a, b.getTimestampSeconds());
        Pose2d poseB = b.getPose();

        // Check inter-camera consistency.
        double separation = previewedPoseA.getTranslation().getDistance(poseB.getTranslation());
        double consistencyMultiplier = consistencyMultiplier(separation);

        double[] varA = squaredStdDevs(a.getStdDevs());
        double[] varB = squaredStdDevs(b.getStdDevs());

        double wxA = 1.0 / varA[0];
        double wyA = 1.0 / varA[1];
        double wtA = 1.0 / varA[2];

        double wxB = 1.0 / varB[0];
        double wyB = 1.0 / varB[1];
        double wtB = 1.0 / varB[2];

        double totalWX = wxA + wxB;
        double totalWY = wyA + wyB;
        double totalWT = wtA + wtB;

        double fusedX = (previewedPoseA.getX() * wxA + poseB.getX() * wxB) / totalWX;
        double fusedY = (previewedPoseA.getY() * wyA + poseB.getY() * wyB) / totalWY;

        Rotation2d fusedHeading;
        boolean aHasTrustedHeading = varA[2] < largeVariance * 0.5;
        boolean bHasTrustedHeading = varB[2] < largeVariance * 0.5;
        if (aHasTrustedHeading && bHasTrustedHeading) {
            fusedHeading = new Rotation2d(
                previewedPoseA.getRotation().getCos() * wtA / totalWT
                    + poseB.getRotation().getCos() * wtB / totalWT,
                previewedPoseA.getRotation().getSin() * wtA / totalWT
                    + poseB.getRotation().getSin() * wtB / totalWT
            );
        } else {
            fusedHeading = bHasTrustedHeading
                ? poseB.getRotation()
                : previewedPoseA.getRotation();
        }

        Pose2d fusedPose = new Pose2d(fusedX, fusedY, fusedHeading);

        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
            Math.sqrt(1.0 / totalWX) * consistencyMultiplier,
            Math.sqrt(1.0 / totalWY) * consistencyMultiplier,
            Math.sqrt(1.0 / totalWT) * consistencyMultiplier
        );

        tagScratch.clear();
            mergeTags(a.getTrackedTags());
            mergeTags(b.getTrackedTags());

        nameBuilder.setLength(0);
        nameBuilder.append("fused[")
            .append(a.getSourceName()).append(", ")
            .append(b.getSourceName()).append(']');

        return Optional.of(new ApriltagResult(
            nameBuilder.toString(),
            b.getTimestampSeconds(),
            Math.max(a.getLatencyMs(), b.getLatencyMs()),
            fusedPose,
            fusedStdDevs,
            tagScratch,
            Math.max(a.getAmbiguity(), b.getAmbiguity()),
            (a.getAverageArea() + b.getAverageArea()) / 2.0
        ));
    }

    /**
     * Previews {@code result}'s pose forward to {@code targetTimestamp} by
     * applying the odometry delta between the two timestamps. If the buffer
     * cannot supply historical poses, the original pose is returned unchanged.
     */
    private Pose2d previewToTimestamp(ApriltagResult result, double targetTimestamp) {
        Optional<Pose2d> atResult = poseBufferSupplier.apply(result.getTimestampSeconds());
        Optional<Pose2d> atTarget = poseBufferSupplier.apply(targetTimestamp);

        if (atResult.isEmpty() || atTarget.isEmpty()) {
            return result.getPose();
        }

        // Delta in odometry from result's timestamp to target's timestamp.
        Transform2d delta = new Transform2d(atResult.get(), atTarget.get());
        return result.getPose().transformBy(delta);
    }

    /**
     * Returns a multiplier to apply to fused stdDevs based on inter-camera agreement.
     */
    private static double consistencyMultiplier(double separationMeters) {
        if (separationMeters <= consistencyThreshold) {
            // Smooth interpolation from AGREEMENT_BONUS at 0 to 1.0 at threshold.
            double t = separationMeters / consistencyThreshold;
            return agreementBonus + (1.0 - agreementBonus) * t;
        } else {
            // Ramp penalty linearly from 1.0 at threshold to DISAGREEMENT_PENALTY at 2× threshold.
            double excess = (separationMeters - consistencyThreshold) / consistencyThreshold;
            return Math.min(1.0 + (disagreementPenalty - 1.0) * excess, disagreementPenalty);
        }
    }

    private static double[] squaredStdDevs(Matrix<N3, N1> stdDevs) {
        double sx = stdDevs.get(0, 0);
        double sy = stdDevs.get(1, 0);
        double st = stdDevs.get(2, 0);
        return new double[]{ sx * sx, sy * sy, st * st };
    }

    private void mergeTags(List<TrackedTag> source) {
        outer:
        for (TrackedTag t : source) {
            for (TrackedTag existing : tagScratch) {
                if (existing.fiducialId == t.fiducialId) continue outer;
            }
            tagScratch.add(t);
        }
    }
    
}