package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.PoseEstimate;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

/**
 * A processor backed by a Limelight camera, using the LimelightHelpers NT API.
 */
public class LimelightProcessor implements ProcessorInterface {

    private final String limelightName;
    private final Transform3d cameraTransform;
    private final StdDevCalculator stdDevCalculator;

    private volatile double cachedHeadingDegrees = 0.0;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue;
    private final AtomicInteger queueSize;
    private final int maxQueueSize;

    private final Notifier processingNotifier;
    private final double processingFrequency;

    private volatile boolean isRunning;
    private volatile boolean useMegaTag2;

    // Reusable scratch list built on the Notifier thread
    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    /**
     * @param limelightName   NT table name (e.g. {@code "limelight-front"});
     *                        empty string uses the default {@code "limelight"} table.
     * @param cameraTransform Static transform from robot centre to camera lens.
     *                        Also configure this in the Limelight web UI or via
     *                        {@link LimelightHelpers#setCameraPose_RobotSpace}.
     * @param calculator      Per-camera std-dev calculator; must not be shared.
     */
    public LimelightProcessor(
        String limelightName,
        Transform3d cameraTransform,
        StdDevCalculator calculator
    ) {
        this.limelightName = limelightName;
        this.cameraTransform = cameraTransform;
        this.stdDevCalculator = calculator;
        this.useMegaTag2 = true;

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.queueSize = new AtomicInteger(0);
        this.maxQueueSize = 5;

        this.processingNotifier  = new Notifier(this::process);
        this.processingFrequency = 30.0;
        this.processingNotifier.setName("Limelight-" + limelightName);
        this.isRunning = false;
    }

    //#region Processing

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(1.0 / processingFrequency);
    }

    @Override
    public void stop() {
        isRunning = false;
        processingNotifier.stop();
        resultQueue.clear();
        queueSize.set(0);
    }

    /**
     * Called periodically on the Notifier thread.
     * <ul>
     *   <li>Sends robot orientation to the Limelight (required for MegaTag2) using
     *       the heading cached by the main thread via {@link #updateHeading(double)}.
     *       This avoids calling into YAGSL from a background thread.</li>
     *   <li>Reads the latest pose estimate from NetworkTables.</li>
     *   <li>Converts it to an {@link ApriltagResult} and enqueues it.</li>
     * </ul>
     */
    @Override
    public void process() {
        if (!isRunning) return;

        if (useMegaTag2) {
            // cachedHeadingDegrees is written by the main thread and read here — volatile
            // guarantees visibility without requiring a lock.
            LimelightHelpers.SetRobotOrientation_NoFlush(limelightName, cachedHeadingDegrees, 0, 0, 0, 0, 0);
        }

        PoseEstimate estimate = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimate == null || estimate.tagCount == 0) return;

        ApriltagResult result = convertToResult(estimate);
        if (result == null) return;

        while (queueSize.get() >= maxQueueSize) {
            if (resultQueue.poll() != null) queueSize.decrementAndGet();
        }
        
        resultQueue.add(result);
        queueSize.incrementAndGet();
    }

    /**
     * Converts a {@link PoseEstimate} into an {@link ApriltagResult}.
     * Returns {@code null} if no usable fiducial data is present.
     */
    private ApriltagResult convertToResult(PoseEstimate estimate) {
        // Limelight area is 0-100%; normalize to 0-1 for StdDevCalculator and filters.
        double avgAreaFraction = estimate.avgTagArea / 100.0;
        double avgAmbiguity = calculateAmbiguity(estimate);
        double latencyMs = estimate.latency;

        tagScratch.clear();
        RawFiducial[] fiducials = estimate.rawFiducials;
        if (fiducials != null) {
            for (int i = 0; i < fiducials.length; i++) {
                RawFiducial f = fiducials[i];
                // Per-tag area isn't individually reported in the botpose NT entry;
                // we use the aggregate average for every tag in this result.
                tagScratch.add(new TrackedTag(f.id, avgAreaFraction, f.ambiguity));
            }
        }

        if (tagScratch.isEmpty()) return null;

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            avgAmbiguity, avgAreaFraction, latencyMs, estimate.tagCount
        );

        return new ApriltagResult(
            limelightName,
            estimate.timestampSeconds,
            latencyMs,
            estimate.pose,
            stdDevs,
            tagScratch,
            avgAmbiguity,
            avgAreaFraction
        );
    }

    /**
     * Produces a representative ambiguity value for the pose estimate.
     * <ul>
     *   <li>MegaTag2: {@code 0.0} (rotation is externally constrained).</li>
     *   <li>MegaTag1, single tag: ambiguity of that fiducial.</li>
     *   <li>MegaTag1, multi-tag: mean ambiguity scaled by sqrt(n)
     * </ul>
     */
    private static double calculateAmbiguity(PoseEstimate estimate) {
        if (estimate.isMegaTag2) return 0.0;

        RawFiducial[] fiducials = estimate.rawFiducials;
        if (fiducials == null || fiducials.length == 0) return 0.0;
        if (fiducials.length == 1) return fiducials[0].ambiguity;

        double sum = 0.0;
        for (RawFiducial f : fiducials) sum += f.ambiguity;

        return (sum / fiducials.length) / Math.sqrt(fiducials.length);
    }

    //#endregion

    //#region Processor

    @Override public String getCameraName() { 
        return limelightName; 
    }

    @Override public Transform3d getCameraTransform() { 
        return cameraTransform; 
    }

    @Override
    public void drainResultQueue(List<ResultInterface> destination) {
        ResultInterface result;
        while ((result = resultQueue.poll()) != null) {
            destination.add(result);
            queueSize.decrementAndGet();
        }
    }

    @Override
    public List<ResultInterface> getResultQueue() {
        List<ResultInterface> results = new ArrayList<>();
        drainResultQueue(results);
        return results;
    }

    @Override public int getMaxQueueSize() { 
        return maxQueueSize; 
    }

    @Override public Notifier getNotifier() { 
        return processingNotifier; 
    }

    @Override public double getFrequency() { 
        return processingFrequency; 
    }

    @Override public boolean isRunning() { 
        return isRunning; 
    }

    @Override
    public void setPipeline(int newPipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, newPipelineIndex);
    }

    /**
     * Not supported. Set the camera pose via the Limelight web UI or
     * {@link LimelightHelpers#setCameraPose_RobotSpace} instead.
     */
    @Override
    public void setCameraTransform(Transform3d newCameraTransform) {
        throw new UnsupportedOperationException(
            "LimelightProcessor: setCameraTransform is not supported at runtime. "
            + "Configure the camera pose via LimelightHelpers.setCameraPose_RobotSpace() "
            + "or the Limelight web UI."
        );
    }

    //#endregion

    //#region Limelight

    /**
     * Pushes the robot's current heading to this processor so MegaTag2 can use it.
     *
     * @param degrees Current robot yaw in degrees (WPILib convention, CCW positive).
     */
    public void updateHeading(double degrees) {
        this.cachedHeadingDegrees = degrees;
    }

    /**
     * Enables or disables MegaTag2 localization.
     * MegaTag2 (gyro-constrained) is on by default. Disable only if the gyro is unreliable.
     */
    public void setUseMegaTag2(boolean useMegaTag2) { 
        this.useMegaTag2 = useMegaTag2; 
    }

    public boolean isUsingMegaTag2() { return useMegaTag2; }

    //#endregion
}