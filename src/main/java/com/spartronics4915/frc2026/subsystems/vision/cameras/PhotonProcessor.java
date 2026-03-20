package com.spartronics4915.frc2026.subsystems.vision.cameras;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Vision processor backed by PhotonVision
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator poseEstimator;

    private final PhotonCameraSim cameraSim;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue;
    private final AtomicInteger queueSize;
    private final int maxQueueSize;

    private final Notifier processingNotifier;
    private final double processingFrequency;

    private volatile boolean isRunning;

    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        SimCameraProperties properties
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.cameraTransform = transform;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraTransform);

        this.cameraSim = new PhotonCameraSim(photonCamera, properties);

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.queueSize = new AtomicInteger(0);
        this.maxQueueSize = maxResultQueueSize;

        this.processingFrequency = 70.0;
        this.processingNotifier  = new Notifier(this::process);
        this.processingNotifier.setName("Photon-" + cameraName);
        this.isRunning = false;
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

        for (PhotonPipelineResult rawResult : photonCamera.getAllUnreadResults()) {
            processApriltagResult(rawResult).ifPresent(result -> {
                // Drop the oldest frame if the consumer can't keep up.
                while (queueSize.get() >= maxQueueSize) {
                    if (resultQueue.poll() != null) queueSize.decrementAndGet();
                }
                resultQueue.add(result);
                queueSize.incrementAndGet();
            });
        }
    }

    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int numTags = targets.size();
        boolean isMultiTag = numTags > 1;

        double avgAmbiguity = computeAvgAmbiguity(targets);
        double avgArea = computeAvgArea(targets);

        // Multi-tag: coprocessor multi-tag solve. Single-tag: lowest-ambiguity solve.
        Optional<EstimatedRobotPose> poseOpt = isMultiTag
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOpt.isEmpty()) return Optional.empty();

        EstimatedRobotPose estimated = poseOpt.get();
        Pose2d robotPose = estimated.estimatedPose.toPose2d();
        double timestamp = estimated.timestampSeconds;
        double latencyMs = rawResult.metadata.getLatencyMillis();

        Matrix<N3, N1> stdDevs = StdDevCalculator.calculate(avgArea, numTags, isMultiTag);

        // For single-tag, capture the raw camera-to-target transform.
        // VisionSubsystem uses this for the gyro-bearing path, which bypasses
        // the ambiguous heading from the pose solve entirely.
        Optional<Transform3d> cameraToTarget = (!isMultiTag && !targets.isEmpty())
            ? Optional.of(targets.get(0).getBestCameraToTarget())
            : Optional.empty();

        tagScratch.clear();
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            tagScratch.add(
                new TrackedTag(
                    target.fiducialId, 
                    target.getArea(), 
                    target.getPoseAmbiguity()
                )
            );
        }

        return Optional.of(new ApriltagResult(
            cameraName,
            timestamp,
            latencyMs,
            robotPose,
            stdDevs,
            tagScratch,
            avgAmbiguity,
            avgArea,
            cameraToTarget,
            isMultiTag
        ));
    }

    private static double computeAvgAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) return targets.get(0).getPoseAmbiguity();
        // Multi-tag: report the best (lowest) individual ambiguity, normalized
        // down by sqrt(N) to reflect that more tags = more constrained solve.
        double best = Double.MAX_VALUE;
        for (PhotonTrackedTarget t : targets) {
            double a = t.getPoseAmbiguity();
            if (a >= 0 && a < best) best = a;
        }
        return (best == Double.MAX_VALUE ? 0.15 : best) / Math.sqrt(targets.size());
    }

    private static double computeAvgArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget t : targets) sum += t.getArea();
        return sum / targets.size();
    }

    @Override public String getCameraName() { 
        return cameraName; 
    }

    @Override public Transform3d getCameraTransform() { 
        return cameraTransform; 
    }

    @Override
    public Optional<PhotonCameraSim> getCameraSim() {
        return Optional.of(cameraSim);
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
    public void setPipeline(int index) {
        photonCamera.setPipelineIndex(index);
    }

    @Override
    public void setCameraTransform(Transform3d newTransform) {
        poseEstimator.setRobotToCameraTransform(newTransform);
    }

}