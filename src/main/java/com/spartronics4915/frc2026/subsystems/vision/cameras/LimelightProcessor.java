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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Processes Limelight AprilTag detections and converts them to pose estimates.
 * Supports both fixed-mount and turreted configurations with MegaTag2/MegaTag1 fusion.
 */
public class LimelightProcessor implements ProcessorInterface {

    private final String name;
    private final double processingFrequencyHz;
    private final int maxQueueSize;

    // Fixed configuration (null if turreted)
    private final Transform3d fixedCameraTransform;

    // Turreted configuration (null if fixed)
    private final Translation3d robotToTurret;
    private final Transform3d turretToCamera;
    private final boolean turreted;

    private final StdDevCalculator stdDevCalculator;

    // Main-thread state (synchronized via volatile)
    private volatile double robotHeadingDegrees = 0.0;
    private volatile boolean useMegaTag2 = true;

    // Turreted-only state: capture timestamps for per-frame interpolation
    private final ConcurrentTimeBuffer<Double> turretYawBuffer;
    private final AtomicReference<Double> latestTurretYawRad;

    // Result queue (Notifier → Main thread)
    private final ConcurrentLinkedQueue<ResultInterface> resultQueue = new ConcurrentLinkedQueue<>();
    private final AtomicInteger queueSize = new AtomicInteger(0);

    private final Notifier processingNotifier;
    private volatile boolean isRunning = false;

    // Scratch objects (Notifier thread only)
    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

    /**
     * Fixed-camera constructor.
     *
     * @param limelightName   NetworkTables table name (e.g. "limelight-front").
     * @param cameraTransform Static robot-to-camera transform.
     * @param calculator      Per-camera std-dev calculator; must not be shared.
     * @param frequencyHz     Processing rate in Hz (typical: 30-50).
     */
    public LimelightProcessor(
        String limelightName,
        Transform3d cameraTransform,
        StdDevCalculator calculator,
        double frequencyHz
    ) {
        this.name = limelightName;
        this.fixedCameraTransform = cameraTransform;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.robotToTurret = null;
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
     * @param limelightName      NetworkTables table name.
     * @param robotToTurretPivot Translation from robot center to turret rotation axis.
     * @param turretToCamera     Static transform from turret pivot to camera (turret frame).
     * @param calculator         Per-camera std-dev calculator; must not be shared.
     * @param frequencyHz        Processing rate in Hz (typical: 100 for Limelight 4).
     */
    public LimelightProcessor(
        String limelightName,
        Translation3d robotToTurretPivot,
        Transform3d turretToCamera,
        StdDevCalculator calculator,
        double frequencyHz
    ) {
        this.name = limelightName;
        this.robotToTurret = robotToTurretPivot;
        this.turretToCamera = turretToCamera;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.fixedCameraTransform = null;
        this.turreted = true;
        this.turretYawBuffer = ConcurrentTimeBuffer.createDoubleBuffer(turretHistorySeconds);
        this.latestTurretYawRad = new AtomicReference<>(0.0);

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("LimelightTurret-" + limelightName);
    }

    @Override
    public void start() {
        isRunning = true;
        if (turreted) {
            initializeCameraPose();
        }
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
            processRotating();
        } else {
            processFixed();
        }
    }

    /**
     * Initialize Limelight camera geometry (turreted only).
     *
     * <p>Pushes only Z-height and fixed pitch/roll; X, Y, yaw are zero so the
     * Limelight returns camera-in-field pose. Per-frame turret angle correction
     * (in {@link #correctPoseForTurretAngle}) recovers the true robot pose.
     */
    private void initializeCameraPose() {
        double cameraZ = robotToTurret.getZ() + turretToCamera.getTranslation().getZ();
        Rotation3d camRotation = turretToCamera.getRotation();

        LimelightHelpers.setCameraPose_RobotSpace(
            name,
            0.0, 0.0, cameraZ,
            Math.toDegrees(camRotation.getX()),
            Math.toDegrees(camRotation.getY()),
            0.0
        );
    }

    private void processFixed() {
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                name, 
                robotHeadingDegrees, 
                0, 0, 0, 0, 0
            );
        }

        PoseEstimate estimate = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (estimate == null || estimate.tagCount == 0) return;
        convertToQueue(estimate, estimate.pose);
    }

    private void processRotating() {
        // For MegaTag2, pass effective heading = robot_heading + turret_yaw + camera_fixed_yaw
        // so Limelight derives correct camera orientation (even though stored yaw is zero)
        if (useMegaTag2) {
            double currentTurretYaw = latestTurretYawRad.get();
            double camFixedYaw = turretToCamera.getRotation().getZ();
            double effectiveHeadingDeg = robotHeadingDegrees
                + Math.toDegrees(currentTurretYaw + camFixedYaw);
            LimelightHelpers.SetRobotOrientation_NoFlush(
                name,
                effectiveHeadingDeg,
                0, 0, 0, 0, 0
            );
        }

        // Primary: MegaTag2 (zero ambiguity, gyro-constrained)
        PoseEstimate mt2 = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
            : null;

        if (mt2 != null && mt2.tagCount > 0) {
            Pose2d corrected = correctPoseForAngle(mt2.pose, mt2.timestampSeconds);
            convertToQueue(mt2, corrected);
            return;
        }

        // Fallback: MegaTag1 (full 6-DOF, useful with multiple tags)
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (mt1 != null && mt1.tagCount > 0) {
            Pose2d corrected = correctPoseForAngle(mt1.pose, mt1.timestampSeconds);
            convertToQueue(mt1, corrected);
        }
    }

    /**
     * Recover true robot pose from Limelight's camera-in-field estimate.
     *
     * <p>Limelight is initialized with X = 0, Y = 0, yaw = 0, so it returns the camera's
     * pose in field coordinates. Apply the inverse of the full robot-to-camera
     * transform (which depends on turret angle) to recover the true robot pose.
     *
     * @param cameraPose       Pose returned by Limelight (camera-in-field).
     * @param captureTimestamp FPGA capture timestamp to interpolate turret angle.
     * @return True robot pose in field coordinates.
     */
    private Pose2d correctPoseForAngle(Pose2d cameraPose, double captureTimestamp) {
        // Interpolate turret angle from buffer, fall back to latest if unavailable
        double turretYawRad = turretYawBuffer
            .getSample(captureTimestamp)
            .orElse(latestTurretYawRad.get());

        Transform2d robotToCamera2d = computeRobotToCamera(turretYawRad);
        return cameraPose.transformBy(robotToCamera2d.inverse());
    }

    /**
     * Build 2D robot-to-camera transform for the given turret yaw.
     *
     * <p>Computes the camera's position and orientation relative to the robot center
     * when the turret is at the specified angle. Z-height is ignored (handled in init).
     *
     * @param turretYawRadians Robot-relative turret yaw (CCW positive).
     */
    private Transform2d computeRobotToCamera(double turretYawRadians) {
        double cosYaw = Math.cos(turretYawRadians);
        double sinYaw = Math.sin(turretYawRadians);
        double camLocalX = turretToCamera.getTranslation().getX();
        double camLocalY = turretToCamera.getTranslation().getY();
        return new Transform2d(
            robotToTurret.getX() + camLocalX * cosYaw - camLocalY * sinYaw,
            robotToTurret.getY() + camLocalX * sinYaw + camLocalY * cosYaw,
            new Rotation2d(turretYawRadians + turretToCamera.getRotation().getZ())
        );
    }

    /**
     * Build full 3D robot-to-camera transform (for {@link #getCameraTransform()}).
     * Internal pose correction uses the 2D overload instead.
     *
     * @param turretYawRadians Robot-relative turret yaw (CCW positive).
     */
    private Transform3d computeRobotToCamera3d(double turretYawRadians) {
        Rotation3d yaw3d = new Rotation3d(0, 0, turretYawRadians);
        Translation3d offset = turretToCamera.getTranslation().rotateBy(yaw3d);
        return new Transform3d(
            robotToTurret.plus(offset),
            yaw3d.plus(turretToCamera.getRotation())
        );
    }

    /**
     * Convert Limelight PoseEstimate to ApriltagResult and enqueue.
     *
     * @param estimate      Raw Limelight result (metadata: timestamp, latency, tags).
     * @param correctedPose True robot pose after turret correction (or estimate.pose if fixed).
     */
    private void convertToQueue(PoseEstimate estimate, Pose2d correctedPose) {
        double avgAreaFraction = estimate.avgTagArea / 100.0;
        double avgAmbiguity = calculateAmbiguity(estimate);
        double latencyMs = estimate.latency;

        // Extract tracked tags from raw fiducials
        tagScratch.clear();
        RawFiducial[] fiducials = estimate.rawFiducials;
        if (fiducials != null) {
            for (RawFiducial fiducial : fiducials) {
                tagScratch.add(new TrackedTag(fiducial.id, avgAreaFraction, fiducial.ambiguity));
            }
        }

        if (tagScratch.isEmpty()) return;

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            avgAmbiguity, avgAreaFraction, latencyMs, estimate.tagCount);

        // Scale down multi-tag std devs in turreted mode
        if (turreted && estimate.tagCount >= 2) {
            stdDevs = scaleMultiTagStdDevs(stdDevs, estimate.tagCount);
        }

        enqueue(new ApriltagResult(
            name, estimate.timestampSeconds, latencyMs,
            correctedPose, stdDevs, tagScratch, 
            avgAmbiguity, avgAreaFraction
        ));
    }

    private static double calculateAmbiguity(PoseEstimate estimate) {
        if (estimate.isMegaTag2) return 0.0;

        RawFiducial[] fiducials = estimate.rawFiducials;
        if (fiducials == null || fiducials.length == 0) return 0.0;
        if (fiducials.length == 1) return fiducials[0].ambiguity;

        double sum = 0.0;
        for (RawFiducial fiducial : fiducials) sum += fiducial.ambiguity;
        return (sum / fiducials.length) / Math.sqrt(fiducials.length);
    }

    /** Scale std devs down by 1/√n for multi-tag turreted observations. */
    private static Matrix<N3, N1> scaleMultiTagStdDevs(Matrix<N3, N1> std, int n) {
        double s = 1.0 / Math.sqrt(n);
        return VecBuilder.fill(
            std.get(0, 0) * s, 
            std.get(1, 0) * s, 
            std.get(2, 0) * s
        );
    }

    private void enqueue(ApriltagResult result) {
        // Drop oldest result if queue is full
        while (queueSize.get() >= maxQueueSize) {
            if (resultQueue.poll() != null) queueSize.decrementAndGet();
        }
        resultQueue.add(result);
        queueSize.incrementAndGet();
    }

    /** Queue size scales with frequency to handle bursts without dropping. */
    private static int computeMaxQueueSize(double hz) {
        return Math.max(4, (int) Math.ceil(hz / 15.0));
    }

    @Override
    public String getCameraName() { 
        return name; 
    }

    @Override
    public boolean isTurreted() { 
        return turreted; 
    }

    @Override
    public Transform3d getCameraTransform() {
        if (!turreted) return fixedCameraTransform;
        return computeRobotToCamera3d(latestTurretYawRad.get());
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

    @Override
    public int getMaxQueueSize() { 
        return maxQueueSize; 
    }

    @Override
    public Notifier getNotifier() { 
        return processingNotifier; 
    }

    @Override
    public double getFrequency() { 
        return processingFrequencyHz; 
    }

    @Override
    public boolean isRunning() { 
        return isRunning; 
    }

    @Override
    public void setPipeline(int idx) {
        LimelightHelpers.setPipelineIndex(name, idx);
    }

    @Override
    public void setCameraTransform(Transform3d t) {
        if (turreted) {
            throw new UnsupportedOperationException(
                "LimelightProcessor (turreted): transform is dynamic; adjust turretToCamera at construction."
            );
        }
    }

    /**
     * {@inheritDoc}
     *
     * <p>In turreted mode, records the sample for per-frame interpolation.
     * In fixed mode, this is a no-op.
     */
    @Override
    public void updateHeading(Rotation2d turretAngle, double timestamp) {
        if (!turreted) return;
        double rad = turretAngle.getRadians();
        latestTurretYawRad.set(rad);
        turretYawBuffer.addSample(timestamp, rad);
    }

    /** Enable/disable MegaTag2 gyro-constrained localization. */
    public void setUseMegaTag2(boolean use) { 
        this.useMegaTag2 = use; 
    }

    public boolean isUsingMegaTag2() { 
        return useMegaTag2; 
    }

}