package com.spartronics4915.frc2026.subsystems.vision.cameras;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;

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
 * Processes Limelight AprilTag detections with zero-allocation pooling.
 */
public class LimelightProcessor implements ProcessorInterface {

    private final String name;
    private final double processingFrequencyHz;
    private final int maxQueueSize;

    private final Transform3d fixedCameraTransform;
    private final Translation3d robotToTurret;
    private final Transform3d turretToCamera;
    private final boolean turreted;

    private final StdDevCalculator stdDevCalculator;

    private volatile double robotHeadingDegrees = 0.0;
    private volatile boolean useMegaTag2 = true;

    private final ConcurrentTimeBuffer<Double> yawBuffer;
    private volatile double latestYawRad = 0.0;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue = new ConcurrentLinkedQueue<>();
    private final AtomicInteger queueSize = new AtomicInteger(0);

    private final Notifier processingNotifier;
    private volatile boolean isRunning = false;

    private final List<TrackedTag> tagScratch = new ArrayList<>(maxTagsPerFrame);
    private final TrackedTag[][] tagCache;
    private final ApriltagResult[] resultCache;
    private int resultCacheIndex = 0;

    private final Matrix<N3, N1> stdDevScratch = VecBuilder.fill(0.0, 0.0, 0.0);
    private final Matrix<N3, N1> scaledStdDevScratch = VecBuilder.fill(0.0, 0.0, 0.0);

    public LimelightProcessor(String limelightName, Transform3d cameraTransform, StdDevCalculator calculator, double frequencyHz) {
        this.name = limelightName;
        this.fixedCameraTransform = cameraTransform;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.robotToTurret = null;
        this.turretToCamera = null;
        this.turreted = false;
        this.yawBuffer = null;

        this.resultCache = allocateResultCache(maxQueueSize);
        this.tagCache = allocateTagCache(maxQueueSize);

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("Limelight-" + limelightName);
    }

    public LimelightProcessor(String limelightName, Translation3d robotToTurretPivot, Transform3d turretToCamera, StdDevCalculator calculator, double frequencyHz) {
        this.name = limelightName;
        this.robotToTurret = robotToTurretPivot;
        this.turretToCamera = turretToCamera;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.fixedCameraTransform = null;
        this.turreted = true;
        this.yawBuffer = ConcurrentTimeBuffer.createDoubleBuffer(turretHistorySeconds);

        this.resultCache = allocateResultCache(maxQueueSize);
        this.tagCache = allocateTagCache(maxQueueSize);

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("LimelightTurret-" + limelightName);
    }

    private static ApriltagResult[] allocateResultCache(int size) {
        ApriltagResult[] cache = new ApriltagResult[size];
        for (int i = 0; i < size; i++) cache[i] = new ApriltagResult();
        return cache;
    }

    private static TrackedTag[][] allocateTagCache(int size) {
        TrackedTag[][] cache = new TrackedTag[size][maxTagsPerFrame];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < maxTagsPerFrame; j++) {
                cache[i][j] = new TrackedTag(0, 0, 0);
            }
        }
        return cache;
    }

    @Override
    public void start() {
        isRunning = true;
        if (turreted) initializeCameraPose();
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
        if (turreted) processRotating();
        else processFixed();
    }

    private void initializeCameraPose() {
        double cameraZ = robotToTurret.getZ() + turretToCamera.getTranslation().getZ();
        Rotation3d camRotation = turretToCamera.getRotation();

        LimelightHelpers.setCameraPose_RobotSpace(
            name, 
            0.0, 
            0.0, 
            cameraZ, 
            Math.toDegrees(camRotation.getX()), 
            Math.toDegrees(camRotation.getY()), 
            Math.toDegrees(camRotation.getZ())
        );
    }

    private void processFixed() {
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(name, robotHeadingDegrees, 0, 0, 0, 0, 0);
        }
        PoseEstimate estimate = useMegaTag2 ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (estimate == null || estimate.tagCount == 0) return;
        convertToQueue(estimate, estimate.pose);
    }

    private void processRotating() {
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation_NoFlush(name, robotHeadingDegrees, 0, 0, 0, 0, 0);
        }

        PoseEstimate mt2 = useMegaTag2 ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) : null;
        if (mt2 != null && mt2.tagCount > 0) {
            convertToQueue(mt2, correctPoseForAngle(mt2.pose, mt2.timestampSeconds));
            return;
        }

        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        if (mt1 != null && mt1.tagCount > 0) {
            convertToQueue(mt1, correctPoseForAngle(mt1.pose, mt1.timestampSeconds));
        }
    }

    private Pose2d correctPoseForAngle(Pose2d cameraPose, double captureTimestamp) {
        double turretYawRad = yawBuffer.getSample(captureTimestamp).orElse(latestYawRad);
        return cameraPose.transformBy(computeRobotToCamera(turretYawRad).inverse());
    }

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

    private void convertToQueue(PoseEstimate estimate, Pose2d correctedPose) {
        double avgAreaFraction = estimate.avgTagArea / 100.0;
        double avgAmbiguity = calculateAmbiguity(estimate);
        double latencyMs = estimate.latency;

        tagScratch.clear();
        RawFiducial[] fiducials = estimate.rawFiducials;
        TrackedTag[] currentFrameTags = tagCache[resultCacheIndex];
        
        if (fiducials != null) {
            int count = Math.min(fiducials.length, maxTagsPerFrame);
            for (int i = 0; i < count; i++) {
                currentFrameTags[i].set(fiducials[i].id, avgAreaFraction, fiducials[i].ambiguity);
                tagScratch.add(currentFrameTags[i]);
            }
        }
        if (tagScratch.isEmpty()) return;

        Matrix<N3, N1> calculatedStd = stdDevCalculator.calculate(avgAmbiguity, avgAreaFraction, latencyMs, estimate.tagCount);
        for(int i = 0; i < 3; i++) stdDevScratch.set(i, 0, calculatedStd.get(i, 0));

        Matrix<N3, N1> finalStdDevs;
        if (turreted && estimate.tagCount >= 2) {
            fillScaledMultiTagStdDevs(stdDevScratch, scaledStdDevScratch, estimate.tagCount);
            finalStdDevs = scaledStdDevScratch;
        } else {
            finalStdDevs = stdDevScratch;
        }

        ApriltagResult pooledResult = resultCache[resultCacheIndex];
        resultCacheIndex = (resultCacheIndex + 1) % maxQueueSize;

        pooledResult.set(
            name, 
            estimate.timestampSeconds, 
            latencyMs, 
            correctedPose, 
            finalStdDevs, 
            tagScratch, 
            avgAmbiguity, 
            avgAreaFraction
        );

        queue(pooledResult);
    }

    private static void fillScaledMultiTagStdDevs(Matrix<N3, N1> source, Matrix<N3, N1> destination, int n) {
        double s = 1.0 / Math.sqrt(n);
        destination.set(0,0, source.get(0,0)*s);
        destination.set(1,0, source.get(1,0)*s);
        destination.set(2,0, source.get(2,0)*s);
    }

    private void queue(ApriltagResult result) {
        while (queueSize.get() >= maxQueueSize) {
            if (resultQueue.poll() != null) queueSize.decrementAndGet();
        }
        resultQueue.add(result);
        queueSize.incrementAndGet();
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

    private static int computeMaxQueueSize(double hz) { return Math.max(4, (int) Math.ceil(hz / 15.0)); }

    @Override public String getCameraName() { 
        return name; 
    }

    @Override public boolean isTurreted() { 
        return turreted; 
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
        return computeRobotToCamera3d(latestYawRad);
    }

    private Transform3d computeRobotToCamera3d(double turretYawRadians) {
        Rotation3d yaw3d = new Rotation3d(0, 0, turretYawRadians);
        return new Transform3d(robotToTurret.plus(turretToCamera.getTranslation().rotateBy(yaw3d)), yaw3d.plus(turretToCamera.getRotation()));
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

    @Override public void setPipeline(int idx) { 
        LimelightHelpers.setPipelineIndex(name, idx); 
    }

    @Override public void setCameraTransform(Transform3d transform) {
        if (turreted) throw new UnsupportedOperationException("Turreted transform is dynamic.");
    }

    @Override
    public void updateHeading(Rotation2d turretAngle, double timestamp) {
        if (!turreted) return;
        double rad = turretAngle.getRadians();
        latestYawRad = rad;
        yawBuffer.addSample(timestamp, rad);
    }

    @Override
    public void setRobotHeading(double headingDegrees) {
        this.robotHeadingDegrees = headingDegrees;
    }

    @Override
    public void updateTurretAngle(Rotation2d turretAngle, double timestamp) {
        if (!turreted) return;
        double rad = turretAngle.getRadians();
        latestYawRad = rad;
        yawBuffer.addSample(timestamp, rad);
    }

    public void setUseMegaTag2(boolean use) { 
        this.useMegaTag2 = use; 
    }

}