package com.spartronics4915.frc2026.subsystems.vision.cameras;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;
import com.spartronics4915.frc2026.util.vision.ConcurrentTimeBuffer;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.PoseEstimate;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

/**
 * A processor backed by a Limelight camera using the LimelightHelpers NT API.
 */
public class LimelightProcessor implements ProcessorInterface {

    private final String limelightName;
    private final double processingFrequencyHz;
    private final int maxQueueSize;

    // Fixed-camera fields (null when turreted)
    private final Transform3d fixedCameraTransform;

    // Turreted-camera fields (null when fixed)
    private final Translation3d robotToTurretPivot;
    private final Transform3d turretToCamera;
    private final boolean turreted;

    private final StdDevCalculator stdDevCalculator;

    /** Robot heading written by the main thread, read by the Notifier thread. */
    private volatile double cachedHeadingDegrees = 0.0;

    /** MegaTag2 enabled flag — disable only when gyro is unreliable. */
    private volatile boolean useMegaTag2 = true;

    /** Turret yaw history for per-frame interpolation (turreted mode only). */
    private final ConcurrentTimeBuffer<Double> turretYawBuffer;

    /** Latest turret yaw — fast fallback when buffer is empty (turreted mode only). */
    private final AtomicReference<Double> latestTurretYawRad;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue = new ConcurrentLinkedQueue<>();
    private final AtomicInteger queueSize = new AtomicInteger(0);

    private final Notifier processingNotifier;
    private volatile boolean isRunning = false;

    /** Scratch list reused on the Notifier thread only. */
    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    /**
     * Fixed-camera constructor.
     *
     * @param limelightName    NT table name (e.g. {@code "limelight-front"}).
     * @param cameraTransform  Static robot-to-camera transform.
     * @param calculator       Per-camera std-dev calculator; must not be shared.
     * @param frequencyHz      Processing rate (Hz).  Typical: 30–50 for fixed
     *                         cameras; up to 100 for a Limelight 4.
     */
    public LimelightProcessor(
        String limelightName,
        Transform3d cameraTransform,
        StdDevCalculator calculator,
        double frequencyHz
    ) {
        this.limelightName = limelightName;
        this.fixedCameraTransform = cameraTransform;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.robotToTurretPivot = null;
        this.turretToCamera = null;
        this.turreted = false;
        this.turretYawBuffer = null;
        this.latestTurretYawRad = null;

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("Limelight-" + limelightName);
    }

    /**
     * Turreted-camera constructor.
     *
     * <p>The robot-to-camera transform is recomputed each frame from the current
     * turret yaw.  Call {@link #updateTurretAngle} and {@link #updateHeading}
     * every robot loop iteration.
     *
     * @param limelightName      NT table name (e.g. {@code "limelight-turret"}).
     * @param robotToTurretPivot Translation from robot centre to the turret rotation axis.
     * @param turretToCamera     Static transform from the turret pivot to the camera
     *                           lens, expressed in the turret's local frame.
     * @param calculator         Per-camera std-dev calculator; must not be shared.
     * @param frequencyHz        Processing rate (Hz). Use 100 for a Limelight 4.
     */
    public LimelightProcessor(
        String limelightName,
        Translation3d robotToTurretPivot,
        Transform3d turretToCamera,
        StdDevCalculator calculator,
        double frequencyHz
    ) {
        this.limelightName = limelightName;
        this.robotToTurretPivot = robotToTurretPivot;
        this.turretToCamera = turretToCamera;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.fixedCameraTransform = null;
        this.turreted = true;
        this.turretYawBuffer = ConcurrentTimeBuffer.createDoubleBuffer(turretHistorySeconds);
        this.latestTurretYawRad   = new AtomicReference<>(0.0);

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("LimelightTurret-" + limelightName);
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

        if (turreted) {
            processTurreted();
        } else {
            processFixed();
        }
    }

    private void processFixed() {
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                limelightName, 
                cachedHeadingDegrees, 
                0, 
                0, 
                0, 
                0, 
                0
            );
        }

        PoseEstimate estimate = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimate == null || estimate.tagCount == 0) return;
        convertToQueue(estimate, fixedCameraTransform);
    }

    private void processTurreted() {
        double turretYawRad = latestTurretYawRad.get();
        Transform3d robotToCamera = computeRobotToCamera(turretYawRad);

        // Push dynamic camera pose so the LL's internal solver uses correct geometry.
        pushCameraPoseToLimelight(robotToCamera);

        // Push robot heading for MT2 gyro constraint.
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                limelightName, 
                cachedHeadingDegrees, 
                0, 
                0, 
                0, 
                0, 
                0
            );
        }

        // Primary: MegaTag2 (zero ambiguity, gyro-constrained).
        PoseEstimate mt2 = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : null;
            
        if (mt2 != null && mt2.tagCount > 0) {
            convertToQueue(mt2, interpolateTurretTransform(mt2.timestampSeconds));
            return;
        }

        // Fallback: MegaTag1 (full 6-DOF, useful with multiple tags).
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 != null && mt1.tagCount > 0) {
            convertToQueue(mt1, interpolateTurretTransform(mt1.timestampSeconds));
        }
    }

    private void convertToQueue(PoseEstimate estimate, Transform3d robotToCamera) {
        double avgAreaFraction = estimate.avgTagArea / 100.0;
        double avgAmbiguity = calculateAmbiguity(estimate);
        double latencyMs = estimate.latency;

        tagScratch.clear();
        RawFiducial[] fiducials = estimate.rawFiducials;
        if (fiducials != null) {
            for (RawFiducial fiducial : fiducials) {
                tagScratch.add(
                    new TrackedTag(
                        fiducial.id, 
                        avgAreaFraction, 
                        fiducial.ambiguity
                    )
                );
            }
        }

        if (tagScratch.isEmpty()) return;

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            avgAmbiguity, avgAreaFraction, latencyMs, estimate.tagCount);

        if (turreted && estimate.tagCount >= 2) {
            stdDevs = scaleMultiTagStdDevs(stdDevs, estimate.tagCount);
        }

        enqueue(new ApriltagResult(
            limelightName,
            estimate.timestampSeconds,
            latencyMs,
            estimate.pose,
            stdDevs,
            tagScratch,
            avgAmbiguity,
            avgAreaFraction
        ));
    }

    /**
     * Builds the robot-to-camera transform for the given turret yaw.
     * The turret rotates about the Z axis; the camera is rigidly attached to it.
     *
     * @param turretYawRadians Robot-relative turret yaw (CCW positive).
     */
    private Transform3d computeRobotToCamera(double turretYawRadians) {
        Rotation3d turretYaw3d = new Rotation3d(0, 0, turretYawRadians);
        Translation3d cameraOffset = turretToCamera.getTranslation().rotateBy(turretYaw3d);
        return new Transform3d(
            robotToTurretPivot.plus(cameraOffset),
            turretYaw3d.plus(turretToCamera.getRotation())
        );
    }

    /**
     * Returns the robot-to-camera transform interpolated to the given capture timestamp.
     * Falls back to the latest known yaw if no buffer entry exists yet.
     */
    private Transform3d interpolateTurretTransform(double captureTimestamp) {
        double yawRad = turretYawBuffer
            .getSample(captureTimestamp)
            .orElse(latestTurretYawRad.get());
        return computeRobotToCamera(yawRad);
    }

    /**
     * Pushes the dynamic camera-in-robot-space pose to the Limelight.
     * Convention: forward = robot +X, side = +Y, up = +Z; angles in degrees.
     */
    private void pushCameraPoseToLimelight(Transform3d robotToCamera) {
        Translation3d transform = robotToCamera.getTranslation();
        Rotation3d rotation = robotToCamera.getRotation();
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            transform.getX(), transform.getY(), transform.getZ(),
            Math.toDegrees(rotation.getX()),
            Math.toDegrees(rotation.getY()),
            Math.toDegrees(rotation.getZ())
        );
    }

    private static double calculateAmbiguity(PoseEstimate estimate) {
        if (estimate.isMegaTag2) return 0.0;

        RawFiducial[] fiducial = estimate.rawFiducials;
        if (fiducial == null || fiducial.length == 0) return 0.0;
        if (fiducial.length == 1) return fiducial[0].ambiguity;

        double sum = 0.0;
        for (RawFiducial raw : fiducial) sum += raw.ambiguity;
        return (sum / fiducial.length) / Math.sqrt(fiducial.length);
    }

    /** Scales std devs down by 1/√n for multi-tag turreted observations. */
    private static Matrix<N3, N1> scaleMultiTagStdDevs(Matrix<N3, N1> std, int n) {
        double s = 1.0 / Math.sqrt(n);
        return VecBuilder.fill(
            std.get(0, 0) * s, 
            std.get(1, 0) * s, 
            std.get(2, 0) * s
        );
    }

    private void enqueue(ApriltagResult result) {
        while (queueSize.get() >= maxQueueSize) {
            if (resultQueue.poll() != null) queueSize.decrementAndGet();
        }
        resultQueue.add(result);
        queueSize.incrementAndGet();
    }

    /** Queue size scales with frequency so we don't drop fast bursts. */
    private static int computeMaxQueueSize(double hz) {
        return Math.max(4, (int) Math.ceil(hz / 15.0));
    }

    @Override public String getCameraName() { 
        return limelightName; 
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
    public void drainResultQueue(List<ResultInterface> destination) {
        ResultInterface r;
        while ((r = resultQueue.poll()) != null) {
            destination.add(r);
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
    public void setPipeline(int idx) {
        LimelightHelpers.setPipelineIndex(limelightName, idx);
    }

    @Override
    public void setCameraTransform(Transform3d t) {
        if (turreted) throw new UnsupportedOperationException(
            "LimelightProcessor (turreted): transform is dynamic; adjust turretToCamera at construction."
        );
    }

    /**
     * Pushes the current robot heading for MegaTag2 gyro-constraint.
     *
     * @param degrees Gyro yaw in degrees (CCW positive, WPILib convention).
     */
    public void updateHeading(double degrees) {
        this.cachedHeadingDegrees = degrees;
    }

    /**
     * {@inheritDoc}
     *
     * <p>In turreted mode this records the sample in a {@link ConcurrentTimeBuffer}
     * so the Notifier thread can interpolate the angle back to each frame's exact
     * capture timestamp.  In fixed mode this is a no-op.
     */
    @Override
    public void updateTurretAngle(Rotation2d turretAngle, double timestamp) {
        if (!turreted) return;
        double rad = turretAngle.getRadians();
        latestTurretYawRad.set(rad);
        turretYawBuffer.addSample(timestamp, rad);
    }

    /** Enables or disables MegaTag2 gyro-constrained localization. */
    public void setUseMegaTag2(boolean use) { 
        this.useMegaTag2 = use; 
    }

    public boolean isUsingMegaTag2() { 
        return useMegaTag2; 
    }

}
