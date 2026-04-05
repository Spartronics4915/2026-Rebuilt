package com.spartronics4915.frc2026.subsystems.vision.cameras;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

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
    private final AtomicReference<Double> latestTurretYawRad;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue = new ConcurrentLinkedQueue<>();
    private final AtomicInteger queueSize = new AtomicInteger(0);

    private final Notifier processingNotifier;
    private volatile boolean isRunning = false;

    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    /**
     * Fixed-camera constructor.
     *
     * @param name          PhotonVision camera name.
     * @param layout        AprilTag field layout.
     * @param transform     Static robot-to-camera transform.
     * @param calculator    Per-camera std-dev calculator; must not be shared.
     * @param simProperties Sim camera properties ({@code null} disables sim).
     * @param frequencyHz   Processing rate (Hz).
     */
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
        this.latestTurretYawRad = null;

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("Photon-" + cameraName);
    }

    /**
     * Turreted-camera constructor.
     *
     * <p>{@link PhotonPoseEstimator#setRobotToCameraTransform} is updated before
     * every estimation call using the turret angle interpolated from a time buffer.
     *
     * @param name               PhotonVision camera name.
     * @param layout             AprilTag field layout.
     * @param robotToTurretPivot Translation from robot centre to turret pivot.
     * @param turretToCamera     Static transform from turret pivot to camera (turret frame).
     * @param calculator         Per-camera std-dev calculator; must not be shared.
     * @param simProperties      Sim camera properties ({@code null} disables sim).
     * @param frequencyHz        Processing rate (Hz).
     */
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

        // Initialize with identity; updated dynamically each frame.
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, computeRobotToCamera(0.0));
        this.cameraSim = (simProperties != null) 
            ? new PhotonCameraSim(photonCamera, simProperties) 
            : null;

        this.fixedCameraTransform = null;
        this.turreted = true;
        this.turretYawBuffer = ConcurrentTimeBuffer.createDoubleBuffer(turretHistorySeconds);
        this.latestTurretYawRad = new AtomicReference<>(0.0);

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("PhotonTurret-" + cameraName);
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

        for (PhotonPipelineResult rawResult : rawResults) {
            // In turreted mode, update the pose estimator's transform to match
            // the turret angle at this frame's exact capture timestamp.
            if (turreted) {
                double captureTs = rawResult.getTimestampSeconds();
                double yawRad = turretYawBuffer
                    .getSample(captureTs)
                    .orElse(latestTurretYawRad.get());
                poseEstimator.setRobotToCameraTransform(computeRobotToCamera(yawRad));
            }

            processApriltagResult(rawResult).ifPresent(
                result -> {
                    while (queueSize.get() >= maxQueueSize) {
                        if (resultQueue.poll() != null) queueSize.decrementAndGet();
                    }
                    resultQueue.add(result);
                    queueSize.incrementAndGet();
                }
            );
        }
    }

    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();
        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);

        Optional<EstimatedRobotPose> poseOpt = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOpt.isEmpty()) return Optional.empty();

        EstimatedRobotPose estPose = poseOpt.get();
        Pose2d resultPose = estPose.estimatedPose.toPose2d();
        double timestamp = Utils.fpgaToCurrentTime(estPose.timestampSeconds);
        double latency = rawResult.metadata.getLatencyMillis();

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(avgAmbiguity, avgArea, latency, targetCount);

        if (turreted && targetCount >= 2) {
            stdDevs = scaleMultiTagStdDevs(stdDevs, targetCount);
        }

        tagScratch.clear();
        for (PhotonTrackedTarget target : targets) {
            tagScratch.add(
                new TrackedTag(
                    target.fiducialId, 
                    target.getArea(), 
                    target.getPoseAmbiguity()
                )
            );
        }

        return Optional.of(
            new ApriltagResult(
                cameraName, 
                timestamp, 
                latency,
                resultPose, 
                stdDevs,
                tagScratch, 
                avgAmbiguity, 
                avgArea
            )
        );
    }

    private Transform3d computeRobotToCamera(double turretYawRadians) {
        Rotation3d yaw3d = new Rotation3d(0, 0, turretYawRadians);
        Translation3d offset = turretToCamera.getTranslation().rotateBy(yaw3d);
        return new Transform3d(
            robotToTurretPivot.plus(offset),
            yaw3d.plus(turretToCamera.getRotation())
        );
    }

    private static double calculateAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) return targets.get(0).getPoseAmbiguity();
        double min = 0.0;
        for (PhotonTrackedTarget t : targets) {
            double a = t.getPoseAmbiguity();
            if (a >= 0 && a < min) min = a;
        }
        return min / Math.sqrt(targets.size());
    }

    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget t : targets) sum += t.getArea();
        return sum / targets.size();
    }

    private static Matrix<N3, N1> scaleMultiTagStdDevs(Matrix<N3, N1> std, int n) {
        double s = 1.0 / Math.sqrt(n);
        return VecBuilder.fill(
            std.get(0, 0) * s, 
            std.get(1, 0) * s, 
            std.get(2, 0) * s
        );
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

    @Override
    public Transform3d getCameraTransform() {
        if (!turreted) return fixedCameraTransform;
        return computeRobotToCamera(latestTurretYawRad.get());
    }

    @Override
    public Optional<PhotonCameraSim> getCameraSim() {
        return Optional.ofNullable(cameraSim);
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
        return processingFrequencyHz; 
    }

    @Override public boolean isRunning() {
        return isRunning; 
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

    /**
     * {@inheritDoc}
     *
     * In turreted mode, records the sample for per-frame interpolation.
     * In fixed mode, this is a no-op.
     */
    @Override
    public void updateHeading(Rotation2d turretAngle, double timestamp) {
        if (!turreted) return;
        double rad = turretAngle.getRadians();
        latestTurretYawRad.set(rad);
        turretYawBuffer.addSample(timestamp, rad);
    }

}
