package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;

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
import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

/**
 * A processor that utilizes photon vision's pre-existing camera class.
 * The processor keeps a linked queue containing its most recent results
 * which can be grabbed and used for further use.
 *
 * <p>Each instance owns a {@link StdDevCalculator} that maintains smoothed
 * distance and area state across frames. This means std devs are computed
 * here, camera-by-camera, rather than centrally in VisionSubsystem.
 *
 * <p>All per-frame calculation methods use plain loops rather than stream
 * pipelines to avoid iterator and lambda allocation on the hot Notifier path.
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;

    private final SimCameraProperties simProperties;
    private final PhotonCameraSim cameraSim;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue;
    private final int maxQueueSize;
    private final Notifier processingNotifier;
    private final double processingFrequency;

    private volatile boolean isRunning;

    private Supplier<ChassisSpeeds> robotVelocitySupplier;

    /**
     * Constructs a photon processor with a set of parameters
     *
     * @param name the name of the processor
     * @param layout the apriltag field layout of the current field
     * @param transform the transform from the center of the robot to the camera lens
     * @param calculator per-camera std dev calculator; must not be shared between cameras
     * @param properties the simulator properties for the camera
     * @param chassisSpeedsSupplier the supplier for the robot's field-relative speeds
     */
    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        StdDevCalculator calculator,
        SimCameraProperties properties,
        Supplier<ChassisSpeeds> chassisSpeedsSupplier
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.cameraTransform = transform;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraTransform);
        this.stdDevCalculator = calculator;

        this.simProperties = properties;
        this.cameraSim = new PhotonCameraSim(photonCamera, simProperties);

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.maxQueueSize = 4;

        this.processingNotifier = new Notifier(this::process);
        this.processingFrequency = 50.0;
        this.processingNotifier.setName("Spectrum-" + cameraName);
        this.isRunning = false;

        this.robotVelocitySupplier = chassisSpeedsSupplier;
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
    }

    @Override
    public void process() {
        if (!isRunning) return;

        List<PhotonPipelineResult> rawResults = photonCamera.getAllUnreadResults();

        for (PhotonPipelineResult rawResult : rawResults) {
            Optional<ApriltagResult> apriltagResult = processApriltagResult(rawResult);
            if (apriltagResult.isPresent()) {
                while (resultQueue.size() >= maxQueueSize) {
                    resultQueue.poll();
                }
                resultQueue.add(apriltagResult.get());
            }
        }
    }

    /**
     * Processes a photon vision pipeline result into an {@link ApriltagResult},
     * including std dev calculation via this camera's {@link StdDevCalculator}.
     *
     * @param rawResult a raw photon vision pipeline result given by the camera
     * @return an optional of a fully populated {@link ApriltagResult}, including std devs
     */
    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();

        double avgDistance = calculateAverageDistance(targets);
        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);
        double xAnisotropy = calculateXAnisotropy(targets);
        double yAnisotropy = calculateYAnisotropy(targets);

        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        if (robotVelocity == null) {
            robotVelocity = new ChassisSpeeds();
        }

        Optional<EstimatedRobotPose> poseOptional = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOptional.isEmpty()) return Optional.empty();
        EstimatedRobotPose estimatedPose = poseOptional.get();
        Pose2d resultantPose = estimatedPose.estimatedPose.toPose2d();

        double timestamp = estimatedPose.timestampSeconds;
        double latency = rawResult.metadata.getLatencyMillis();

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            avgDistance,
            avgAmbiguity,
            avgArea,
            xAnisotropy,
            yAnisotropy,
            robotVelocity,
            latency,
            targetCount
        );

        return Optional.of(new ApriltagResult(
            cameraName,
            timestamp,
            latency,
            resultantPose,
            stdDevs,
            targets,
            avgDistance,
            avgAmbiguity,
            avgArea,
            xAnisotropy,
            yAnisotropy,
            robotVelocity
        ));
    }

    //#endregion

    //#region Calculation

    /**
     * Calculates the average distance to a list of photon vision targets.
     * Uses a plain loop instead of a stream to avoid iterator allocation on the hot path.
     */
    private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sum += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return sum / targets.size();
    }

    /**
     * Calculates the average ambiguity from a list of photon vision targets.
     * Uses a plain loop instead of a stream to avoid iterator allocation on the hot path.
     * For multi-tag results, returns the minimum non-negative ambiguity scaled by tag count.
     */
    private static double calculateAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) {
            return targets.get(0).getPoseAmbiguity();
        }

        double minAmbiguity = 0.15;
        for (PhotonTrackedTarget target : targets) {
            double a = target.getPoseAmbiguity();
            if (a >= 0 && a < minAmbiguity) minAmbiguity = a;
        }
        return minAmbiguity / Math.sqrt(targets.size());
    }

    /**
     * Calculates the average area (size in frame) of a list of photon vision targets.
     * Uses a plain loop instead of a stream to avoid iterator allocation on the hot path.
     */
    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sum += target.getArea();
        }
        return sum / targets.size();
    }

    /**
     * Calculates the average x anisotropy (yaw-based uncertainty)
     * from a list of photon vision targets.
     * Uses a plain loop instead of a stream to avoid iterator allocation on the hot path.
     */
    private static double calculateXAnisotropy(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 1.0;
        double sumAbsYaw = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sumAbsYaw += Math.abs(target.getYaw());
        }
        double avgYawRad = Math.toRadians(sumAbsYaw / targets.size());
        return 1.0 / Math.max(Math.cos(avgYawRad), MIN_COSINE_VALUE);
    }

    /**
     * Calculates the average y anisotropy (pitch-based uncertainty)
     * from a list of photon vision targets.
     * Uses a plain loop instead of a stream to avoid iterator allocation on the hot path.
     */
    private static double calculateYAnisotropy(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 1.0;
        double sumAbsPitch = 0.0;
        for (PhotonTrackedTarget target : targets) {
            sumAbsPitch += Math.abs(target.getPitch());
        }
        double avgPitchRad = Math.toRadians(sumAbsPitch / targets.size());
        return 1.0 / Math.max(Math.cos(avgPitchRad), MIN_COSINE_VALUE);
    }

    //#endregion

    //#region Getters

    @Override
    public String getCameraName() {
        return cameraName;
    }

    @Override
    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    @Override
    public AprilTagFieldLayout getFieldlayout() {
        return fieldLayout;
    }

    @Override
    public Transform3d getCameraTransform() {
        return cameraTransform;
    }

    @Override
    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    @Override
    public SimCameraProperties getSimProperties() {
        return simProperties;
    }

    @Override
    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    /**
     * Drains all pending results from the queue into the provided destination list.
     * Prefer this over {@link #getResultQueue()} to avoid allocating a new list per call.
     *
     * @param destination the list to drain results into; existing contents are preserved
     */
    @Override
    public void drainResultQueue(List<ResultInterface> destination) {
        ResultInterface result;
        while ((result = resultQueue.poll()) != null) {
            destination.add(result);
        }
    }

    /**
     * Convenience wrapper that drains the queue into a new list and returns it.
     * Use {@link #drainResultQueue(List)} instead in performance-sensitive paths.
     */
    @Override
    public List<ResultInterface> getResultQueue() {
        List<ResultInterface> results = new ArrayList<>();
        drainResultQueue(results);
        return results;
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
        return processingFrequency;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public Supplier<ChassisSpeeds> getSpeedSupplier() {
        return robotVelocitySupplier;
    }

    //#endregion

    //#region Setters

    @Override
    public void setPipeline(int newPipelineIndex) {
        photonCamera.setPipelineIndex(newPipelineIndex);
    }

    @Override
    public void setCameraTransform(Transform3d newCameraTransform) {
        poseEstimator.setRobotToCameraTransform(newCameraTransform);
    }

    @Override
    public void setSpeedSupplier(Supplier<ChassisSpeeds> newSupplier) {
        this.robotVelocitySupplier = newSupplier;
    }

    //#endregion
}