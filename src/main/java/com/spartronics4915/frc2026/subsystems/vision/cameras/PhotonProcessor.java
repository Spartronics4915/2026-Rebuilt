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

import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;

/**
 * A processor that utilizes photon vision's pre-existing camera class. 
 * The processor keeps a linked queue containing its most recent results 
 * which can be grabbed and used for further use.
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator poseEstimator;

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
     * @param layout the apriltag field layout of the current 
     * @param transform the transform from the center of the robot to the cameras lens
     * @param properties the simulator properties for the camera
     * @param chassisSpeedsSupplier the supplier for the robots field relative speeds
     */
    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        SimCameraProperties properties,
        Supplier<ChassisSpeeds> chassisSpeedsSupplier
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.cameraTransform = transform;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraTransform);
        
        this.simProperties = properties;
        this.cameraSim = new PhotonCameraSim(photonCamera, simProperties);

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.maxQueueSize = 10;
        
        this.processingNotifier = new Notifier(this::process);
        this.processingFrequency = 100.0;
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
                } resultQueue.add(apriltagResult.get());
            }
        }
    }

    /**
     * Processes a photon vision pipeline result and turns it into a {@link ApriltagResult} 
     * without the standard deviations.
     *  
     * @param rawResult a raw photon vision pipeline result given by the camera
     * @return an optional of a {@link ApriltagResult} 
     */
    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();

        /* 
         * Calculating the different aspects of the camera result 
         * enables the filtering of bad poses + we can use them
         * for standard deviation calculation later on
         */
        double avgDistance = calculateAverageDistance(targets);
        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);
        double x_anisotropy = calculateXAnisotropy(targets);
        double y_anisotropy = calculateYAnisotropy(targets);

        /* 
         * Grabbing the robots velocity at the timestamp of the pose 
         * allows the system to take into account motion blur 
         * for standard deviation calculation later on
         */
        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        if (robotVelocity == null) {
            robotVelocity = new ChassisSpeeds();
        }

        Optional<EstimatedRobotPose> poseOptional = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);
        
        /* 
         * Make sure to do a null safety check here so we dont
         * End up causing a full system crash
         */
        if (poseOptional.isEmpty()) return Optional.empty();
        EstimatedRobotPose estimatedPose = poseOptional.get();
        Pose2d resultantPose = estimatedPose.estimatedPose.toPose2d();

        /*
         * Get the timestamp here so we can pass it to the 
         * kalman filter used by addVisionMeasurement later
         */
        double timestamp = estimatedPose.timestampSeconds;
        double latency = rawResult.metadata.getLatencyMillis();

        /*
         * Return the optional of all of the previous collected data 
         * from the result, also standard deviations are null for now 
         * as we will calculate them later on
         */
        return Optional.of(new ApriltagResult(
            cameraName, 
            timestamp, 
            latency, 
            resultantPose,
            null, 
            targets, 
            avgDistance, 
            avgAmbiguity, 
            avgArea, 
            x_anisotropy,
            y_anisotropy,
            robotVelocity
        ));
    }

    //#endregion

    //#region Calculation
    
    /**
     * Calculates the average distance to a list of photon vision targets
     * 
     * @param targets the targets from a list of photon tracked targets
     * @return the average distance to the list of targets
     */
    private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);
    }

    /**
     * Calculates the average ambiguity from a list of photon vision targets
     * 
     * @param targets the targets from a list of photon tracked targets
     * @return the average ambiguity from the list of targets
     */
    private static double calculateAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) {
            return targets.get(0).getPoseAmbiguity();
        }
    
        double minAmbiguity = targets.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .filter(a -> a >= 0)
            .min()
            .orElse(0.15);

        return minAmbiguity / Math.sqrt(targets.size());
    }

    
    /**
     * Calculates the average area (size in frame) of a list of photon vision targets
     * 
     * @param targets the targets from a list of photon tracked targets
     * @return the average area from the list of targets
     */
    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(PhotonTrackedTarget::getArea)
            .average()
            .orElse(0.0);
    }
    
    /**
     * Calculates the average x anisotropy (y-tilt in camera frame)
     * from a list of photon vision targets
     * 
     * @param targets the targets from a list of photon tracked targets
     * @return the average x anisotropy from the list of targets
     */
    private static double calculateXAnisotropy(List<PhotonTrackedTarget> targets) {
        double avgYawRad = Math.toRadians(
            targets.stream()
                .mapToDouble(target -> Math.abs(target.getYaw()))
                .average()
                .orElse(0.0)
        );
        return 1.0 / Math.max(Math.cos(avgYawRad), MIN_COSINE_VALUE);
    }

    /**
     * Calculates the average y anisotropy (x-tilt in camera frame)
     * from a list of photon vision targets
     * 
     * @param targets the targets from a list of photon tracked targets
     * @return the average y anisotropy from the list of targets
     */
    private static double calculateYAnisotropy(List<PhotonTrackedTarget> targets) {
        double avgPitchRad = Math.toRadians(
            targets.stream()
                .mapToDouble(target -> Math.abs(target.getPitch()))
                .average()
                .orElse(0.0)
        );
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

    @Override
    public List<ResultInterface> getResultQueue() {
        List<ResultInterface> results = new ArrayList<>();
        ResultInterface result;
        while ((result = resultQueue.poll()) != null) {
            results.add(result);
        }
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
