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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;

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
    private final int processingFrequency;

    private volatile boolean isRunning;

    private Supplier<ChassisSpeeds> robotVelocitySupplier;

    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        SimCameraProperties properties,
        Supplier<ChassisSpeeds> vroomSupplier
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
        this.processingFrequency = 100;
        this.processingNotifier.setName("Spectrum-" + cameraName);
        this.isRunning = false;

        this.robotVelocitySupplier = vroomSupplier;
    }

    //#region Processing

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(0.01);
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
                if (resultQueue.size() >= maxQueueSize) {
                    resultQueue.poll();
                }
                resultQueue.add(apriltagResult.get());
            }
        }
    }

    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();

        // Calculate the average distance
        double avgDistance = calculateAverageDistance(targets);

        // Calculate the average ambiguity
        double avgAmbiguity = calculateAverageAmbiguity(targets);

        // Calculate the average area of the frame
        double avgArea = calculateAverageArea(targets);

        // Calculate the anisotropy for x vs. y uncertainty
        double x_anisotropy = calculateXAnisotropy(targets);
        double y_anisotropy = calculateYAnisotropy(targets);

        // Get the robots current velocity
        ChassisSpeeds robotVelocity = robotVelocitySupplier.get();
        if (robotVelocity == null) {
            robotVelocity = new ChassisSpeeds();
        }

        // Gets the vision pose from the result
        EstimatedRobotPose estimatedPose = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult).get()
            : poseEstimator.estimateLowestAmbiguityPose(rawResult).get();

        // Gets the pose2d from the estimated robot pose
        Pose2d resultPose = estimatedPose.estimatedPose.toPose2d();

        // Get the timestamp of the vision pose
        double timestamp = estimatedPose.timestampSeconds;

        return Optional.of(new ApriltagResult(
            cameraName, 
            timestamp, 
            targetCount, 
            resultPose,
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
     * Calculate average distance across all targets
     */
    private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);
    }

    /**
     * Calculate average pose ambiguity across all targets
     */
    private static double calculateAverageAmbiguity(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .filter(a -> a >= 0)
            .average()
            .orElse(0.15);
    }
    
    /**
     * Calculate average tag area (percentage of frame)
     */
    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .mapToDouble(PhotonTrackedTarget::getArea)
            .average()
            .orElse(0.0);
    }
    
    /**
     * Calculate anisotropy factor for X
     */
    private static double calculateXAnisotropy(List<PhotonTrackedTarget> targets) {
        double avgYawRad = Math.toRadians(
            targets.stream()
                .mapToDouble(target -> Math.abs(target.getYaw()))
                .average()
                .orElse(0.0)
        );
    
        // Foreshortening in X direction due to horizontal viewing angle
        return 1.0 / (Math.cos(avgYawRad) + 0.01);
    }

    /**
     * Calculate anisotropy factor for X
     */
    private static double calculateYAnisotropy(List<PhotonTrackedTarget> targets) {
        double avgPitchRad = Math.toRadians(
            targets.stream()
                .mapToDouble(target -> Math.abs(target.getPitch()))
                .average()
                .orElse(0.0)
        );
    
        // Foreshortening in Y direction due to vertical viewing angle
        return 1.0 / (Math.cos(avgPitchRad) + 0.01);
    }

    //#endregion

    //#region Getters

    /**
     * Gets the camera's name
     */
    @Override
    public String getCameraName() {
        return cameraName;
    }

    /**
     * Gets the camera's associated photon camera
     */
    @Override
    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    /**
     * Gets the camera's apriltag field layout
     */
    @Override
    public AprilTagFieldLayout getFieldlayout() {
        return fieldLayout;
    }

    /**
     * Gets the camera's transform from the robots center
     */
    @Override
    public Transform3d getCameraTransform() {
        return cameraTransform;
    }

    /**
     * Gets the camera's associated pose estimator
     */
    @Override
    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Gets the camera's simulator properties
     */
    @Override
    public SimCameraProperties getSimProperties() {
        return simProperties;
    }

    /**
     * Gets the camera's associated simulation for the camera
     */
    @Override
    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    /**
     * Gets the camera's queue of results
     */
    @Override
    public List<ResultInterface> getResultQueue() {
        return new ArrayList<>(resultQueue);
    }

    /**
     * Gets the camera's max size for it's result queue
     */
    @Override
    public int getMaxQueueSize() {
        return maxQueueSize;
    }

    /**
     * Gets the camera's notifier (parallel processing)
     */
    @Override
    public Notifier getNotifier() {
        return processingNotifier;
    }

    /**
     * Gets the camera's processing frequency
     */
    @Override
    public double getFrequency() {
        return processingFrequency;
    }

    /**
     * Gets if the camera's notifier is running
     */
    @Override
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Gets the camera's robot speed supplier
     */
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
