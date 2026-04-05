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

    /**
     * The static camera transform pushed to the Limelight once at {@link #start()}.
     */
    private Transform3d startupCameraTransform = null;

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
     * @param frequencyHz      Processing rate (Hz).  Typical: 30-50 for fixed
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
        if (turreted) {
            initializeLimelightCameraPose();
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
            processTurreted();
        } else {
            processFixed();
        }
    }

    /**
     * Configures the Limelight once at startup with the camera's static geometry.
     *
     * <p>Only the Z height (turret pivot Z + camera Z offset) and the camera's fixed
     * pitch and roll are pushed.  X, Y, and yaw are left at zero — "based on the
     * robot's centre" — so the Limelight's returned robot-pose is equivalent to the
     * camera's pose in field coordinates.  The true robot pose is recovered per-frame
     * in {@link #correctPoseForTurretAngle}.
     */
    private void initializeLimelightCameraPose() {
        double cameraZ = robotToTurretPivot.getZ() + turretToCamera.getTranslation().getZ();
        Rotation3d camRotation = turretToCamera.getRotation();

        // Mirror exactly what is pushed to the Limelight so the correction math
        // in correctPoseForTurretAngle uses the same reference frame.
        startupCameraTransform = new Transform3d(
            new Translation3d(0.0, 0.0, cameraZ),
            new Rotation3d(camRotation.getX(), camRotation.getY(), 0.0)
        );

        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            0.0, 0.0, cameraZ,
            Math.toDegrees(camRotation.getX()),
            Math.toDegrees(camRotation.getY()),
            0.0
        );
    }

    private void processFixed() {
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                limelightName, 
                cachedHeadingDegrees, 
                0, 0, 0, 0, 0
            );
        }

        PoseEstimate estimate = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (estimate == null || estimate.tagCount == 0) return;
        convertToQueue(estimate, estimate.pose);
    }

    private void processTurreted() {
        // By passing (robot_heading + turret_yaw + cam_fixed_yaw) as the "robot
        // heading", MegaTag2 derives the correct camera orientation even though the
        // stored camera-in-robot yaw is zero.
        if (useMegaTag2) {
            double currentTurretYaw = latestTurretYawRad.get();
            double camFixedYaw = turretToCamera.getRotation().getZ();
            double effectiveHeadingDeg = cachedHeadingDegrees
                + Math.toDegrees(currentTurretYaw + camFixedYaw);
            LimelightHelpers.SetRobotOrientation_NoFlush(
                limelightName,
                effectiveHeadingDeg,
                0, 0, 0, 0, 0
            );
        }

        // Primary: MegaTag2 (zero ambiguity, gyro-constrained).
        PoseEstimate mt2 = useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : null;

        if (mt2 != null && mt2.tagCount > 0) {
            Pose2d corrected = correctPoseForTurretAngle(mt2.pose, mt2.timestampSeconds);
            convertToQueue(mt2, corrected);
            return;
        }

        // Fallback: MegaTag1 (full 6-DOF, useful with multiple tags).
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 != null && mt1.tagCount > 0) {
            Pose2d corrected = correctPoseForTurretAngle(mt1.pose, mt1.timestampSeconds);
            convertToQueue(mt1, corrected);
        }
    }

    /**
     * Recovers the true robot pose from the Limelight's estimate.
     *
     * <p>Because the Limelight was initialized with X = 0, Y = 0, yaw = 0 (the
     * camera treated as being at the robot centre), its robot-pose output equals
     * the camera's pose in field coordinates (X/Y/heading). This method applies
     * the inverse of the full robot-to-camera transform built by chaining the
     * robot-to-turret translation and the turret-to-camera transform at the
     * turret angle interpolated back to the exact capture timestamp. To convert
     * from camera-in-field back to robot-in-field.
     *
     * @param cameraPose    Pose returned by the Limelight (treated as camera global pose).
     * @param captureTimestamp FPGA capture timestamp in seconds.
     * @return True robot pose in WPILib blue-origin field coordinates.
     */
    private Pose2d correctPoseForTurretAngle(Pose2d cameraPose, double captureTimestamp) {
        // Use the turret angle that was active when the frame was captured.
        // Falls back to the latest known angle if the buffer doesn't reach that far.
        double turretYawRad = turretYawBuffer
            .getSample(captureTimestamp)
            .orElse(latestTurretYawRad.get());

        Transform2d robotToCamera2d = computeRobotToCamera(turretYawRad);

        // limelightPose ~= camera world pose. Applying the inverse recovers robot pose.
        return cameraPose.transformBy(robotToCamera2d.inverse());
    }

    /**
     * Builds the 2-D robot-to-camera transform for the given turret yaw.

     * <p>Only X, Y, and yaw are computed, Z is irrelevant for pose correction
     * and is handled exclusively in {@link #initializeLimelightCameraPose}.
     *
     * @param turretYawRadians Robot-relative turret yaw (CCW positive).
     */
    private Transform2d computeRobotToCamera(double turretYawRadians) {
        double cosYaw = Math.cos(turretYawRadians);
        double sinYaw = Math.sin(turretYawRadians);
        double camLocalX = turretToCamera.getTranslation().getX();
        double camLocalY = turretToCamera.getTranslation().getY();
        return new Transform2d(
            robotToTurretPivot.getX() + camLocalX * cosYaw - camLocalY * sinYaw,
            robotToTurretPivot.getY() + camLocalX * sinYaw + camLocalY * cosYaw,
            new Rotation2d(turretYawRadians + turretToCamera.getRotation().getZ())
        );
    }

    /**
     * Builds the full 3-D robot-to-camera transform for the given turret yaw.
     *
     * <p>Only used by {@link #getCameraTransform()} to satisfy the
     * {@link ProcessorInterface} contract. All internal pose-correction logic
     * uses the 2-D overload {@link #computeRobotToCamera(double)} instead.
     *
     * @param turretYawRadians Robot-relative turret yaw (CCW positive).
     */
    private Transform3d computeRobotToCamera3d(double turretYawRadians) {
        Rotation3d turretYaw3d = new Rotation3d(0, 0, turretYawRadians);
        Translation3d cameraOffset = turretToCamera.getTranslation().rotateBy(turretYaw3d);
        return new Transform3d(
            robotToTurretPivot.plus(cameraOffset),
            turretYaw3d.plus(turretToCamera.getRotation())
        );
    }

    /**
     * Converts a Limelight {@link PoseEstimate} into an {@link ApriltagResult} and
     * enqueues it.
     *
     * @param estimate      Raw Limelight estimate supplying metadata (timestamp,
     *                      latency, area, fiducials).
     * @param correctedPose The true robot pose after turret-angle correction has been
     *                      applied (for fixed cameras this is {@code estimate.pose}
     *                      unchanged).
     */
    private void convertToQueue(PoseEstimate estimate, Pose2d correctedPose) {
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
            correctedPose,
            stdDevs,
            tagScratch,
            avgAmbiguity,
            avgAreaFraction
        ));
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

    /** Scales std devs down by 1/sqrt(n) for multi-tag turreted observations. */
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
     * <p>In turreted mode this records the sample in the {@link ConcurrentTimeBuffer}
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