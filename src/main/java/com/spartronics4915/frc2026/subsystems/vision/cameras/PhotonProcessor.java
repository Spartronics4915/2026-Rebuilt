package com.spartronics4915.frc2026.subsystems.vision.cameras;

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
 * A processor backed by PhotonVision's camera and pose estimator.
 *
 * <p>Supports both fixed cameras and cameras mounted on a rotating turret.
 * Use the standard constructor for a fixed camera, or
 * {@link #createTurreted} for a camera that rotates with the turret.
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;

    private final PhotonCameraSim cameraSim;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue;
    private final AtomicInteger queueSize;
    private final int maxQueueSize;

    private final Notifier processingNotifier;
    private final double processingFrequency;

    private volatile boolean isRunning;

    // Reusable scratch list (Notifier thread only).
    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    // ----- Constructors -----

    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d robotToCamera,
        StdDevCalculator calculator,
        SimCameraProperties properties
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.cameraTransform = robotToCamera;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
        this.stdDevCalculator = calculator;

        this.cameraSim = new PhotonCameraSim(photonCamera, properties);

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.queueSize = new AtomicInteger(0);
        this.maxQueueSize = 6;

        this.processingNotifier = new Notifier(this::process);
        this.processingFrequency = 70.0;
        this.processingNotifier.setName("Photon-" + cameraName);
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

    @Override
    public void process() {
        if (!isRunning) return;

        List<PhotonPipelineResult> rawResults = photonCamera.getAllUnreadResults();

        for (PhotonPipelineResult rawResult : rawResults) {
            Optional<ApriltagResult> apriltagResult = processApriltagResult(rawResult);
            if (apriltagResult.isPresent()) {
                while (queueSize.get() >= maxQueueSize) {
                    if (resultQueue.poll() != null) queueSize.decrementAndGet();
                }
                resultQueue.add(apriltagResult.get());
                queueSize.incrementAndGet();
            }
        }
    }

    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();

        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);

        Optional<EstimatedRobotPose> poseOptional = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOptional.isEmpty()) return Optional.empty();

        EstimatedRobotPose estimatedPose = poseOptional.get();
        double timestamp = estimatedPose.timestampSeconds;
        double latency = rawResult.metadata.getLatencyMillis();

        Pose2d resultantPose = estimatedPose.estimatedPose.toPose2d();

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            avgAmbiguity,
            avgArea,
            latency,
            targetCount
        );

        tagScratch.clear();
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget t = targets.get(i);
            tagScratch.add(new TrackedTag(t.fiducialId, t.getArea(), t.getPoseAmbiguity()));
        }

        return Optional.of(new ApriltagResult(
            cameraName,
            timestamp,
            latency,
            resultantPose,
            stdDevs,
            tagScratch,
            avgAmbiguity,
            avgArea
        ));
    }

    //#endregion

    //#region Calculation

    private static double calculateAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) return targets.get(0).getPoseAmbiguity();

        double minAmbiguity = 0.15;
        for (PhotonTrackedTarget target : targets) {
            double a = target.getPoseAmbiguity();
            if (a >= 0 && a < minAmbiguity) minAmbiguity = a;
        }
        return minAmbiguity / Math.sqrt(targets.size());
    }

    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) sum += target.getArea();
        return sum / targets.size();
    }

    //#endregion

    //#region ProcessorInterface

    @Override public String getCameraName() { 
        return cameraName; 
    }

    @Override public Transform3d getCameraTransform() { 
        return cameraTransform; 
    }

    /** Exposes the PhotonVision sim camera for {@code VisionSystemSim} wiring. */
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
        photonCamera.setPipelineIndex(newPipelineIndex);
    }

    @Override
    public void setCameraTransform(Transform3d newCameraTransform) {
        poseEstimator.setRobotToCameraTransform(newCameraTransform);
    }

    //#endregion
}