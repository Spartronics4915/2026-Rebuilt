package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;
import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult.ResultQuality;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;
import com.spartronics4915.frc2026.util.StdDevCalculator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

public class PhotonVisionCamera implements CameraInterface {
    private final String cameraName;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;
    private final SimCameraProperties simCameraProperties;
    private final PhotonCameraSim cameraSim;
    
    private final ConcurrentLinkedQueue<CameraResult> resultQueue;
    private final Notifier processingNotifier;
    private final int maxQueueSize;

    private volatile boolean isRunning = false;

    public PhotonVisionCamera (
        String cameraName,
        PhotonPoseEstimator poseEstimator,
        SimCameraProperties simCameraProperties,
        double processingFrequencyHz
    ) {
        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = poseEstimator;
        this.stdDevCalculator = new StdDevCalculator();
        this.simCameraProperties = simCameraProperties;
        this.cameraSim = new PhotonCameraSim(camera, simCameraProperties);
        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.maxQueueSize = 10;
        
        this.processingNotifier = new Notifier(this::processInBackground);
        this.processingNotifier.setName("PhotonProcessor-" + cameraName);
    }

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(0.01); // 100Hz
    }

    @Override
    public void stop() {
        isRunning = false;
        processingNotifier.stop();
        resultQueue.clear();
    }

    private void processInBackground() {
        if (!isRunning) return;

        List<PhotonPipelineResult> rawResults = camera.getAllUnreadResults();
        
        for (PhotonPipelineResult rawResult : rawResults) {
            processSingleResult(rawResult);
        }
    }

    @Override
    public List<CameraResult> processFrame(VisionContext context) {
        List<PhotonPipelineResult> rawResults = camera.getAllUnreadResults();
        List<CameraResult> processed = new ArrayList<>(rawResults.size());
        
        for (PhotonPipelineResult rawResult : rawResults) {
            Optional<CameraResult> result = processSingleResultWithContext(rawResult, context);
            result.ifPresent(processed::add);
        }
        
        return processed;
    }

    private void processSingleResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return;

        if (resultQueue.size() >= maxQueueSize) resultQueue.poll();

        double latency = rawResult.metadata.getLatencyMillis();
        double timestamp = Utils.fpgaToCurrentTime(rawResult.getTimestampSeconds());
        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        
        CameraResult result = new CameraResult.Builder()
            .cameraName(cameraName)
            .timestamp(timestamp)
            .latency(latency)
            .targets(targets)
            .build();
            
        resultQueue.offer(result);
    }

    private Optional<CameraResult> processSingleResultWithContext(
        PhotonPipelineResult rawResult, 
        VisionContext context
    ) {
        if (!rawResult.hasTargets()) {
            return Optional.empty();
        }

        VisionConfiguration config = context.getConfig();
        
        double latency = rawResult.metadata.getLatencyMillis();
    
        if (latency > config.maxLatencyMs) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();

        double totalDistance = 0.0;
        for (PhotonTrackedTarget target : targets) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        double avgDistance = totalDistance / targetCount;

        Optional<EstimatedRobotPose> estimatedPose;
        if (targetCount > 1) estimatedPose = poseEstimator.estimateCoprocMultiTagPose(rawResult);
            else estimatedPose = poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (estimatedPose.isEmpty()) return Optional.empty();

        EstimatedRobotPose robotPose = estimatedPose.get();
        Pose2d pose2d = robotPose.estimatedPose.toPose2d();
        double timestamp = Utils.fpgaToCurrentTime(robotPose.timestampSeconds);

        double totalAmbiguity = 0.0;
        for (PhotonTrackedTarget target : targets) totalAmbiguity += target.getPoseAmbiguity();
        double avgAmbiguity = totalAmbiguity / targetCount;

        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            targetCount,
            avgDistance,
            avgAmbiguity,
            config
        );

        // Assess result quality
        ResultQuality quality = assessQuality(
            targetCount,
            avgDistance,
            avgAmbiguity,
            latency,
            config
        );

        return Optional.of(new CameraResult.Builder()
            .cameraName(cameraName)
            .timestamp(timestamp)
            .latency(latency)
            .pose(pose2d)
            .stdDevs(stdDevs)
            .targets(targets)
            .averageDistance(avgDistance)
            .ambiguity(avgAmbiguity)
            .quality(quality)
            .build());
    }

    private ResultQuality assessQuality(
            int targetCount,
            double avgDistance,
            double avgAmbiguity,
            double latency,
            VisionConfiguration config) {
        
        if (avgAmbiguity > config.maxAmbiguityScore) return ResultQuality.POOR;
        if (latency > config.maxLatencyMs * 0.9) return ResultQuality.POOR;
        
        double maxDistance = (targetCount == 1) 
            ? config.maxSingleTagDistanceMeters 
            : config.maxMultiTagDistanceMeters;
        if (avgDistance > maxDistance) return ResultQuality.POOR;

        if (targetCount >= 3 && avgDistance < maxDistance * 0.5 && avgAmbiguity < 0.1) {
            return ResultQuality.EXCELLENT;
        }

        if (targetCount >= 2 && avgDistance < maxDistance * 0.7 && avgAmbiguity < 0.15) {
            return ResultQuality.GOOD;
        }

        return ResultQuality.FAIR;
    }

    @Override
    public PhotonCamera getPhotonCamera() {
        return camera;
    }

    @Override
    public Transform3d getTransform() {
        return poseEstimator.getRobotToCameraTransform();
    }

    @Override
    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    @Override
    public List<CameraResult> getLatestResults() {
        List<CameraResult> results = new ArrayList<>();
        CameraResult result;
        while ((result = resultQueue.poll()) != null) {
            results.add(result);
        }
        return results;
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
    }

    @Override
    public String getCameraName() {
        return cameraName;
    }

    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    public static class Builder {
        private String cameraName;
        private Transform3d robotToCamera;
        private PhotonPoseEstimator poseEstimator;
        private SimCameraProperties simCameraProperties;
        private double processingFrequencyHz = 100.0;

        public Builder cameraName(String name) {
            this.cameraName = name;
            return this;
        }

        public Builder transform(Transform3d robotToCamera) {
            this.robotToCamera = robotToCamera;
            return this;
        }

        public Builder poseEstimator(PhotonPoseEstimator estimator) {
            this.poseEstimator = estimator;
            return this;
        }

        public Builder simCameraProperties(SimCameraProperties simProperties) {
            this.simCameraProperties = simProperties;
            return this;
        }

        public Builder processingFrequency(double hz) {
            this.processingFrequencyHz = hz;
            return this;
        }

        public PhotonVisionCamera build() {
            if (cameraName == null || robotToCamera == null || poseEstimator == null || simCameraProperties == null) {
                throw new IllegalStateException("Camera name, transform, pose estimator, sim camera properties are required");
            }
            return new PhotonVisionCamera(cameraName, poseEstimator, simCameraProperties, processingFrequencyHz);
        }
    }
}
