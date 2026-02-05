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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.util.StdDevCalculator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;

public class PhotonProcessor implements ProcessorInterface {
    private final String cameraName;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;
    private final PhotonCameraSim cameraSim;
    
    private final ConcurrentLinkedQueue<ApriltagResult> resultQueue;
    private final int maxQueueSize;
    private final Notifier processingNotifier;

    private Supplier<ChassisSpeeds> robotVelocitySupplier;
    private VisionConfiguration configuration;

    private volatile boolean isRunning = false;

    public PhotonProcessor(
        String cameraName,
        PhotonPoseEstimator poseEstimator,
        double processingFrequencyHz
    ) {
        this.cameraName = cameraName;
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = poseEstimator;
        this.stdDevCalculator = new StdDevCalculator();
        this.cameraSim = new PhotonCameraSim(camera, VisionConstants.SIM_CAMERA_PROPERTIES);
        this.robotVelocitySupplier = null;
        this.configuration = null;

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.maxQueueSize = 10;
        
        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("PhotonProcessor-" + cameraName);
    }

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(0.006); // 60Hz
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

        List<PhotonPipelineResult> rawResults = camera.getAllUnreadResults();
        
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

    // In PhotonProcessor, update the processApriltagResult method:

    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();

        // Calculate average distance to all targets
        double avgDistance = targets.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0.0);

        // Use multi-tag estimation if possible, otherwise lowest ambiguity
        Optional<EstimatedRobotPose> estimatedPose = (targetCount > 1) 
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult) 
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (estimatedPose.isEmpty()) return Optional.empty();

        EstimatedRobotPose robotPose = estimatedPose.get();
        Pose2d pose2d = robotPose.estimatedPose.toPose2d();
        double timestamp = robotPose.timestampSeconds;

        // Calculate average ambiguity score
        double avgAmbiguity = targets.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .average()
            .orElse(0.0);

        // Get current robot velocity for motion punishment
        ChassisSpeeds robotSpeeds = robotVelocitySupplier != null 
            ? robotVelocitySupplier.get() 
            : new ChassisSpeeds();


        Matrix<N3, N1> stdDevs = stdDevCalculator.calculate(
            targetCount,
            avgDistance,
            avgAmbiguity,
            robotSpeeds,
            configuration
        );

        // Calculate latency
        double latencyMs = rawResult.metadata.getLatencyMillis();

        return Optional.of(new ApriltagResult.Builder()
            .cameraName(cameraName)
            .timestamp(timestamp)
            .latency(latencyMs)
            .pose(pose2d)
            .stdDevs(stdDevs)
            .targets(targets)
            .averageDistance(avgDistance)
            .ambiguity(avgAmbiguity)
            .build());
    }

    @Override
    public void setRobotVelocitySupplier(Supplier<ChassisSpeeds> supplier) {
        this.robotVelocitySupplier = supplier;
    }

    @Override
    public void setVisionConfiguration(VisionConfiguration configuration) {
        this.configuration = configuration;
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

    @Override
    public List<ResultInterface> getResultQueue() {
        return new ArrayList<>(resultQueue);
    }

    public static class Builder {
        private String cameraName;
        private Transform3d robotToCamera;
        private PhotonPoseEstimator poseEstimator;
        private Supplier<ChassisSpeeds> robotVelocitySupplier;
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

        public Builder robotVelocitySupplier(Supplier<ChassisSpeeds> supplier) {
            this.robotVelocitySupplier = supplier;
            return this;
        }

        public Builder processingFrequency(double hz) {
            this.processingFrequencyHz = hz;
            return this;
        }

        public PhotonProcessor build() {
            if (cameraName == null || robotToCamera == null || poseEstimator == null) {
                throw new IllegalStateException("Camera name, transform, and pose estimator are required");
            }
            return new PhotonProcessor(
                cameraName, 
                poseEstimator,  
                processingFrequencyHz
            );
        }
    }
}