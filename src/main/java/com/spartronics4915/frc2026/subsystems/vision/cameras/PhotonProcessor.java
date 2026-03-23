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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
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

    /** Non-null iff this is a turreted camera. Acts as the "is turreted" flag. */
    private final Translation3d robotToTurretTranslation;

    /**
     * History of turret yaw angles keyed by FPGA timestamp.
     * Only ever accessed on the Notifier thread — populated at the top of
     * {@link #process()} from the volatile cache, and sampled per-frame.
     * Null for fixed cameras.
     */
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer;

    /** Latest turret yaw written by the main thread, read by the Notifier thread. */
    private volatile Rotation2d cachedTurretAngle = Rotation2d.kZero;

    /** FPGA timestamp matching {@link #cachedTurretAngle}. */
    private volatile double cachedTurretTimestamp = 0.0;

    // Reusable scratch list (Notifier thread only).
    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    // ----- Constructors -----

    /**
     * Creates a fixed (non-turreted) PhotonVision processor.
     *
     * @param name       PhotonVision camera name (must match the NT table name).
     * @param layout     Field AprilTag layout.
     * @param transform  Static robot-center-to-camera transform.
     * @param calculator Per-camera std-dev calculator; must not be shared.
     * @param properties Simulation camera properties.
     */
    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        StdDevCalculator calculator,
        SimCameraProperties properties
    ) {
        this(name, layout, transform, null, calculator, properties);
    }

    /**
     * Creates a turreted PhotonVision processor via {@link #createTurreted}.
     * Private — callers use the factory method.
     */
    private PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d turretToCameraTransform,
        Translation3d robotToTurretTranslation,
        StdDevCalculator calculator,
        SimCameraProperties properties
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.cameraTransform = turretToCameraTransform;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, turretToCameraTransform);
        this.stdDevCalculator = calculator;
        this.robotToTurretTranslation = robotToTurretTranslation;
        this.turretAngleBuffer = (robotToTurretTranslation != null)
            ? TimeInterpolatableBuffer.createBuffer(1.0)
            : null;

        this.cameraSim = new PhotonCameraSim(photonCamera, properties);

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.queueSize = new AtomicInteger(0);
        this.maxQueueSize = 4;

        this.processingNotifier = new Notifier(this::process);
        this.processingFrequency = 20.0;
        this.processingNotifier.setName("Photon-" + cameraName);
        this.isRunning = false;
    }

    /**
     * Creates a processor for a camera mounted on a rotating turret.
     *
     * <p>Pass the static turret-to-camera transform (camera offset from the
     * turret pivot, ignoring turret yaw) and the translation from the robot
     * center to the turret pivot. Turret yaw is injected each loop via
     * {@link #updateTurretAngle}.
     *
     * @param name                    PhotonVision camera name.
     * @param layout                  Field AprilTag layout.
     * @param turretToCameraTransform Static transform from turret pivot to camera lens.
     * @param robotToTurretTranslation Translation from robot center to turret pivot.
     * @param calculator              Per-camera std-dev calculator.
     * @param properties              Simulation camera properties.
     */
    public static PhotonProcessor createTurreted(
        String name,
        AprilTagFieldLayout layout,
        Transform3d turretToCameraTransform,
        Translation3d robotToTurretTranslation,
        StdDevCalculator calculator,
        SimCameraProperties properties
    ) {
        return new PhotonProcessor(name, layout, turretToCameraTransform,
            robotToTurretTranslation, calculator, properties);
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

        if (turretAngleBuffer != null) {
            turretAngleBuffer.addSample(cachedTurretTimestamp, cachedTurretAngle);
        }

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

        // For a turreted camera, PhotonVision returned a turret-pivot pose in
        // field space. Convert it back to a robot-body pose using the turret yaw
        // that was true when the image was captured.
        Pose2d resultantPose;
        if (robotToTurretTranslation != null) {
            Rotation2d turretYaw = turretAngleBuffer
                .getSample(timestamp)
                .orElse(cachedTurretAngle);

            Pose3d robotPose3d = estimatedPose.estimatedPose.transformBy(
                new Transform3d(
                    robotToTurretTranslation,
                    new Rotation3d(0.0, 0.0, turretYaw.getRadians())
                ).inverse()
            );
            resultantPose = robotPose3d.toPose2d();
        } else {
            resultantPose = estimatedPose.estimatedPose.toPose2d();
        }

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

    /**
     * Records the turret's current yaw for use in per-frame pose correction.
     * No-op for fixed cameras (where {@link #robotToTurretTranslation} is null).
     */
    @Override
    public void updateTurretAngle(Rotation2d turretAngle, double timestamp) {
        if (robotToTurretTranslation == null) return;
        this.cachedTurretAngle = turretAngle;
        this.cachedTurretTimestamp = timestamp;
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