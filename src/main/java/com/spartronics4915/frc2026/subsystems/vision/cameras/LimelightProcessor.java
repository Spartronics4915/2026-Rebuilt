package com.spartronics4915.frc2026.subsystems.vision.cameras;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.PoseEstimate;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Vision processor backed by a Limelight camera via the LimelightHelpers NT API.
 */
public class LimelightProcessor implements ProcessorInterface {

    private final String limelightName;
    private final Transform3d cameraTransform;

    private volatile double cachedHeading = 0.0;
    private volatile double cachedYawRate = 0.0;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue;
    private final AtomicInteger queueSize;

    private final Notifier processingNotifier;
    private static final double processingFrequency = 30.0;

    private volatile boolean isRunning;
    private volatile boolean useMegaTag2 = true;

    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    public LimelightProcessor(String limelightName, Transform3d cameraTransform) {
        this.limelightName = limelightName;
        this.cameraTransform = cameraTransform;

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.queueSize = new AtomicInteger(0);

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("Limelight-" + limelightName);
    }

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

    @Override
    public void process() {
        if (!isRunning) return;

        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                limelightName,
                cachedHeading,
                cachedYawRate,
                0, 0, 0, 0
            );
        }

        PoseEstimate estimate = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimate == null || estimate.tagCount == 0) return;

        ApriltagResult result = convertToResult(estimate);
        if (result == null) return;

        while (queueSize.get() >= maxResultQueueSize) {
            if (resultQueue.poll() != null) queueSize.decrementAndGet();
        }
        resultQueue.add(result);
        queueSize.incrementAndGet();
    }


    private ApriltagResult convertToResult(PoseEstimate estimate) {

        double avgArea = estimate.avgTagArea / 100.0;
        int numTags = estimate.tagCount;
        boolean trustHeading = estimate.isMegaTag2 || numTags > 1;

        tagScratch.clear();
        if (estimate.rawFiducials != null) {
            for (RawFiducial fiducial : estimate.rawFiducials) {
                tagScratch.add(new TrackedTag(fiducial.id, avgArea, fiducial.ambiguity));
            }
        }
        if (tagScratch.isEmpty()) return null;

        double avgAmbiguity = computeAmbiguity(estimate);

        return new ApriltagResult(
            limelightName,
            estimate.timestampSeconds,
            estimate.latency,
            estimate.pose,
            StdDevCalculator.calculate(avgArea, numTags, trustHeading),
            tagScratch,
            avgAmbiguity,
            avgArea,
            Optional.empty(),
            trustHeading
        );
    }

    private static double computeAmbiguity(PoseEstimate estimate) {
        if (estimate.isMegaTag2) return 0.0;

        RawFiducial[] fiducials = estimate.rawFiducials;
        if (fiducials == null || fiducials.length == 0) return 0.0;
        if (fiducials.length == 1) return fiducials[0].ambiguity;

        double sum = 0.0;
        for (RawFiducial fiducial : fiducials) sum += fiducial.ambiguity;
        return (sum / fiducials.length) / Math.sqrt(fiducials.length);
    }

    @Override public String getCameraName() { 
        return limelightName; 
    }

    @Override public Transform3d getCameraTransform() { 
        return cameraTransform; 
    }

    @Override public int getMaxQueueSize() { 
        return maxResultQueueSize; 
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
    public void drainResultQueue(List<ResultInterface> destination) {
        ResultInterface result;
        while ((result = resultQueue.poll()) != null) {
            destination.add(result);
            queueSize.decrementAndGet();
        }
    }

    @Override
    public List<ResultInterface> getResultQueue() {
        List<ResultInterface> out = new ArrayList<>();
        drainResultQueue(out);
        return out;
    }

    @Override
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(limelightName, index);
    }

    @Override
    public void setCameraTransform(Transform3d newTransform) {
        throw new UnsupportedOperationException(
            "LimelightProcessor: setCameraTransform is not supported at runtime. "
            + "Configure the camera pose via the Limelight web UI."
        );
    }

    public void updateHeading(double headingDeg, double yawRateDegS) {
        this.cachedHeading  = headingDeg;
        this.cachedYawRate = yawRateDegS;
    }

    public void updateHeading(double headingDeg) {
        updateHeading(headingDeg, 0.0);
    }

    public void setUseMegaTag2(boolean shouldUse) { 
        useMegaTag2 = shouldUse; 
    }

    public boolean isUsingMegaTag2() { 
        return useMegaTag2; 
    }

}