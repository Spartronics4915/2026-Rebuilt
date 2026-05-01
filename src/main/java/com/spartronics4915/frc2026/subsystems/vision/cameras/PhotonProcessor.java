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

import com.ctre.phoenix6.Utils;
import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;
import com.spartronics4915.frc2026.util.vision.ConcurrentTimeBuffer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

/**
 * A processor backed by PhotonVision's camera and pose estimator.
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;
    private final PhotonCameraSim cameraSim;

    private final double processingFrequencyHz;
    private final int maxQueueSize;

    // Fixed-camera field (null when turreted)
    private final Transform3d fixedCameraTransform;

    // Turreted-camera fields (null when fixed)
    private final Translation3d robotToTurretPivot;
    private final Transform3d turretToCamera;
    private final boolean turreted;

    private final ConcurrentTimeBuffer<Double> turretYawBuffer;
    private volatile double latestTurretYawRad = 0.0; 

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue = new ConcurrentLinkedQueue<>();
    private final AtomicInteger queueSize = new AtomicInteger(0);

    private final Notifier processingNotifier;
    private volatile boolean isRunning = false;

    private final List<TrackedTag> tagList = new ArrayList<>(maxTagsPerFrame);
    private final TrackedTag[][] tagCache;
    private final ApriltagResult[] resultCache;
    private int resultCacheIndex = 0;

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
    private final Matrix<N3, N1> scaledStdDevScratch = VecBuilder.fill(0.0, 0.0, 0.0);

    private double cachedYaw = Double.NaN;
    private Transform3d robotToCamera = new Transform3d();

    /** Fixed-camera constructor. */
    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        StdDevCalculator calculator,
        SimCameraProperties simProperties,
        double frequencyHz
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.fixedCameraTransform = transform;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, transform);
        this.cameraSim = (simProperties != null)
            ? new PhotonCameraSim(photonCamera, simProperties)
            : null;

        this.robotToTurretPivot = null;
        this.turretToCamera = null;
        this.turreted = false;
        this.turretYawBuffer = null;

        this.resultCache = initResultCache(maxQueueSize);
        this.tagCache = new TrackedTag[maxQueueSize][maxTagsPerFrame];
        initTagCache();

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("Photon-" + cameraName);
    }

    /** Turreted-camera constructor. */
    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Translation3d robotToTurretPivot,
        Transform3d turretToCamera,
        StdDevCalculator calculator,
        SimCameraProperties simProperties,
        double frequencyHz
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.robotToTurretPivot = robotToTurretPivot;
        this.turretToCamera = turretToCamera;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, computeRobotToCamera(0.0));
        this.cameraSim = (simProperties != null)
            ? new PhotonCameraSim(photonCamera, simProperties)
            : null;

        this.fixedCameraTransform = null;
        this.turreted = true;
        this.turretYawBuffer = ConcurrentTimeBuffer.createDoubleBuffer(turretHistorySeconds);

        this.resultCache = initResultCache(maxQueueSize);
        this.tagCache = new TrackedTag[maxQueueSize][maxTagsPerFrame];
        initTagCache();

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("PhotonTurret-" + cameraName);
    }

    private static ApriltagResult[] initResultCache(int size) {
        ApriltagResult[] cache = new ApriltagResult[size];
        for (int i = 0; i < size; i++) {
            cache[i] = new ApriltagResult(); 
        }
        return cache;
    }

    private void initTagCache() {
        for (int i = 0; i < maxQueueSize; i++) {
            for (int j = 0; j < maxTagsPerFrame; j++) {
                tagCache[i][j] = new TrackedTag(0, 0.0, 0.0); 
            }
        }
    }

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(1.0 / processingFrequencyHz);
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

        List<PhotonPipelineResult> rawResults = photonCamera.getAllUnreadResults();

        for (int i = 0, n = rawResults.size(); i < n; i++) {
            PhotonPipelineResult rawResult = rawResults.get(i);

            if (turreted) {
                double captureTimestamp = rawResult.getTimestampSeconds();
                double yaw = turretYawBuffer
                    .getSample(captureTimestamp)
                    .orElse(latestTurretYawRad);
                poseEstimator.setRobotToCameraTransform(computeRobotToCamera(yaw));
            }

            ApriltagResult result = processApriltagResult(rawResult);
            if (result != null) {
                while (queueSize.get() >= maxQueueSize) {
                    if (resultQueue.poll() != null) queueSize.decrementAndGet();
                }
                resultQueue.add(result);
                queueSize.incrementAndGet();
            }
        }
    }

    private ApriltagResult processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return null;

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();
        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);

        Optional<EstimatedRobotPose> poseOpt = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOpt.isEmpty()) return null;

        EstimatedRobotPose estimatedRobotPose = poseOpt.get();


        Pose2d poseEstimate = estimatedRobotPose
            .estimatedPose
            .toPose2d();

        double timestamp = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
        double latency = rawResult.metadata.getLatencyMillis();

        Matrix<N3, N1> calculatedValues = stdDevCalculator.calculate(
            avgAmbiguity, avgArea, latency, targetCount
        );

        for (int i = 0; i < 3; i++) {
            stdDevs.set(i, 0, calculatedValues.get(i, 0));
        }

        Matrix<N3, N1> finalStdDevs = stdDevs;
        if (turreted && targetCount >= 2) {
            fillScaledMultiTagStdDevs(stdDevs, scaledStdDevScratch, targetCount);
            finalStdDevs = scaledStdDevScratch;
        }

        TrackedTag[] currentFrameTagCache = tagCache[resultCacheIndex];
            tagList.clear();

        for (int i = 0; i < Math.min(targetCount, maxTagsPerFrame); i++) {
            PhotonTrackedTarget target = targets.get(i);
            currentFrameTagCache[i].set(
                target.getFiducialId(), 
                target.getArea(),
                target.getPoseAmbiguity()
            );
            tagList.add(currentFrameTagCache[i]);
        }

        ApriltagResult result = resultCache[resultCacheIndex];
        resultCacheIndex = (resultCacheIndex + 1) % maxQueueSize;

        result.set(
            cameraName, 
            timestamp, 
            latency, 
            poseEstimate,
            finalStdDevs, 
            tagList, 
            avgAmbiguity, 
            avgArea
        );

        return result;
    }

    private Transform3d computeRobotToCamera(double turretYawRadians) {
        if (!Double.isNaN(cachedYaw) && Math.abs(turretYawRadians - cachedYaw) < yawRecomputeThreshold) {
            return robotToCamera;
        }

        Rotation3d yaw = new Rotation3d(0, 0, turretYawRadians);
        Translation3d offset = turretToCamera.getTranslation().rotateBy(
            yaw
        );

        robotToCamera = new Transform3d(
            robotToTurretPivot.plus(offset),
            yaw.plus(turretToCamera.getRotation())
        );
        cachedYaw = turretYawRadians;
        return robotToCamera;
    }

    private static double calculateAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) return targets.get(0).getPoseAmbiguity();
        // Multi-tag pose estimation is unambiguous (no pose ambiguity)
        return 0.0;
    }

    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (int i = 0, n = targets.size(); i < n; i++) sum += targets.get(i).getArea();
        return sum / targets.size();
    }

    private static void fillScaledMultiTagStdDevs(
        Matrix<N3, N1> source,
        Matrix<N3, N1> destination,
        int n
    ) {
        double s = 1.0 / Math.sqrt(n);
        destination.set(0, 0, source.get(0, 0) * s);
        destination.set(1, 0, source.get(1, 0) * s);
        destination.set(2, 0, source.get(2, 0) * s);
    }

    private static int computeMaxQueueSize(double hz) {
        return Math.max(4, (int) Math.ceil(hz / 10.0));
    }

    @Override public String getCameraName() { 
        return cameraName; 
    }

    @Override public boolean isTurreted() { 
        return turreted; 
    }

    @Override public Optional<PhotonCameraSim> getCameraSim() { 
        return Optional.ofNullable(cameraSim); 
    }

    @Override public int getMaxQueueSize() { 
        return maxQueueSize; 
    }

    @Override public Notifier getNotifier() { 
        return processingNotifier; 
    }

    @Override public double getFrequency() { 
        return processingFrequencyHz; 
    }

    @Override public boolean isRunning() { 
        return isRunning; 
    }

    @Override
    public Transform3d getCameraTransform() {
        if (!turreted) return fixedCameraTransform;
        return computeRobotToCamera(latestTurretYawRad);
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
        photonCamera.setPipelineIndex(index);
    }

    @Override
    public void setCameraTransform(Transform3d t) {
        if (turreted) throw new UnsupportedOperationException(
            "PhotonProcessor (turreted): transform is dynamic; adjust turretToCamera at construction."
        );
        poseEstimator.setRobotToCameraTransform(t);
    }

    @Override
    public void setRobotHeading(double headingDegrees) {
        throw new UnsupportedOperationException(
            "PhotonProcessor does not support adding the robot's current heading"
        );
    }

    @Override
    public void updateTurretAngle(Rotation2d turretAngle, double timestamp) {
        if (!turreted) return;
        double rad = turretAngle.getRadians();
        this.latestTurretYawRad = rad;
        this.turretYawBuffer.addSample(timestamp, rad);
    }

    @Override
    public void updateHeading(Rotation2d turretAngle, double timestamp) {
        updateTurretAngle(turretAngle, timestamp);
    }
    
}