package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext.VisionState;
import com.spartronics4915.frc2026.subsystems.vision.filters.PipelineFilter;
import com.spartronics4915.frc2026.subsystems.vision.filters.ResultFilters;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.util.PerformanceTracker;
import com.spartronics4915.frc2026.util.PoseFusionEngine;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    private final Map<String, ProcessorInterface> cameras;
    public final VisionContext context;
    private final VisionConfiguration config;
    
    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;
    
    private final PipelineFilter aprilTagFilter;
    
    private final PoseFusionEngine fusionEngine;
    private final PerformanceTracker performanceTracker;
    
    private final VisionPoseConsumer poseConsumer;

    private static double lastPoseTimestamp;

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Pose2d> usedPoseSupplier;

    private final StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision Pose", Pose2d.struct).publish();
    private final DoublePublisher translationStdDevPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Translation Std Dev").publish();
    private final DoublePublisher rotationStdDevPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Rotation Std Dev").publish();

    private final StructPublisher<Pose3d> cameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Camera Pose", Pose3d.struct).publish();

    private VisionSubsystem(
        Map<String, ProcessorInterface> cameras,
        AprilTagFieldLayout fieldLayout,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier,
        Supplier<Pose2d> usedPoseSupplier,
        VisionConfiguration config,
        VisionPoseConsumer poseConsumer
    ) {
        this.cameras = cameras;
        this.config = config;
        this.visionSystemSim = new VisionSystemSim("main");
        this.poseConsumer = poseConsumer;
        this.robotPoseSupplier = robotPoseSupplier;
        this.usedPoseSupplier = usedPoseSupplier;
        
        this.context = new VisionContext(
            fieldLayout,
            robotPoseSupplier,
            config,
            VisionState.GLOBAL
        );

        this.aprilTagFilter = PipelineFilter.builder()
            .addFilter(new ResultFilters.HasTargetsFilter())
            .addFilter(new ResultFilters.HasPoseFilter())
            .addFilter(new ResultFilters.LatencyFilter(config.maxLatencyMs))
            .addFilter(new ResultFilters.DistanceFilter(
                config.maxSingleTagDistanceMeters,
                config.maxMultiTagDistanceMeters
            ))
            .addFilter(new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore))
            .build();
        
        this.fusionEngine = new PoseFusionEngine();
        this.performanceTracker = new PerformanceTracker(config.maxPeriodicTimeMs);

        if (robotVelocitySupplier != null) {
            for (ProcessorInterface camera : cameras.values()) {
                camera.setRobotVelocitySupplier(robotVelocitySupplier);
                camera.setVisionConfiguration(config);
            }
        }

        isSimulation = Robot.isSimulation();
        
        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : cameras.values()) {
            camera.start();
            if (isSimulation) visionSystemSim.addCamera(camera.getCameraSim(), camera.getTransform());
        }
    }

    @Override
    public void periodic() {
        performanceTracker.startTiming("periodic_total");
        
        List<ResultInterface> allResults = new ArrayList<>();
        
        for (ProcessorInterface entry : cameras.values()) {
            performanceTracker.startTiming("camera_" + entry.getCameraName());
            allResults.addAll(entry.getResultQueue());
            performanceTracker.stopTiming();
        }
        
        performanceTracker.startTiming("filtering");
        List<ResultInterface> filteredResults = aprilTagFilter.filter(allResults);
        performanceTracker.stopTiming();

        // Convert ResultInterface to ApriltagResult
        List<ApriltagResult> apriltagResults = filteredResults.stream()
            .filter(result -> result instanceof ApriltagResult)
            .map(result -> (ApriltagResult) result)
            .toList();

        // Fuse poses from multiple cameras
        if (!apriltagResults.isEmpty()) {
            performanceTracker.startTiming("pose_fusion");
            
            ApriltagResult fusedResult = fusionEngine.fusePoses(apriltagResults, config);
            
            if (fusedResult != null && fusedResult.hasPose()) {
                lastPoseTimestamp = fusedResult.getTimestampSeconds();
                
                poseConsumer.accept(
                    fusedResult.getEstimatedPose().get(),
                    fusedResult.getTimestampSeconds(),
                    fusedResult.getStdDevs()
                );

                visionPosePublisher.accept(fusedResult.getEstimatedPose().get());
                translationStdDevPublisher.accept(fusedResult.getStdDevs().get(0, 0));
                rotationStdDevPublisher.accept(fusedResult.getStdDevs().get(2, 0));

                cameraPosePublisher.accept(new Pose3d(robotPoseSupplier.get()).plus(VisionConstants.RIGHT_CAMERA_TRANSFORM));
            }
            
            performanceTracker.stopTiming();
        }
        
        if (isSimulation) {
            visionSystemSim.update(robotPoseSupplier.get());
        }
        
        // Stop periodic_total timing
        performanceTracker.stopTiming();
        
        // Publish performance metrics
        performanceTracker.publishMetrics();
    }

    public void setState(VisionState state) {
        context.setState(state);
    }

    public VisionState getState() {
        return context.getCurrentState();
    }

    public static double getPoseTimestamp() {
        return lastPoseTimestamp; 
    }

    public boolean isCameraConnected(String cameraName) {
        ProcessorInterface processor = cameras.get(cameraName);
        return processor != null && processor.isConnected();
    }

    public int getActiveCameraCount() {
        return (int) cameras.values().stream()
            .filter(ProcessorInterface::isConnected)
            .count();
    }

    public void resetPerformanceStats() {
        performanceTracker.reset();
    }

    public boolean isPerformanceHealthy() {
        return performanceTracker.isWithinBudget(config.maxPeriodicTimeMs);
    }

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }

    @FunctionalInterface
    public interface VisionObjectsConsumer {
        void accept(
            Pose2d groupCenter,
            double timestampSeconds
        );
    }

    public static class Builder {
        private final Map<String, ProcessorInterface> cameras = new HashMap<>();
        private AprilTagFieldLayout fieldLayout;
        private Supplier<Pose2d> simPoseSupplier;
        private Supplier<ChassisSpeeds> robotVelocitySupplier;
        private Supplier<Pose2d> usedPoseSupplier;
        private VisionConfiguration config = new VisionConfiguration();
        private VisionPoseConsumer poseConsumer = (pose, time, std) -> {};

        public Builder addCamera(String name, ProcessorInterface processor) {
            cameras.put(name, processor);
            return this;
        }

        public Builder setFieldLayout(AprilTagFieldLayout layout) {
            this.fieldLayout = layout;
            return this;
        }

        public Builder setSimPoseSupplier(Supplier<Pose2d> supplier) {
            this.simPoseSupplier = supplier;
            return this;
        }

        public Builder setRobotVelocitySupplier(Supplier<ChassisSpeeds> supplier) {
            this.robotVelocitySupplier = supplier;
            return this;
        }

        public Builder setUsedPoseSupplier(Supplier<Pose2d> supplier) {
            this.usedPoseSupplier = supplier;
            return this;
        }

        public Builder setConfiguration(VisionConfiguration config) {
            this.config = config;
            return this;
        }

        public Builder setPoseConsumer(VisionPoseConsumer consumer) {
            this.poseConsumer = consumer;
            return this;
        }

        public VisionSubsystem build() {
            if (fieldLayout == null || simPoseSupplier == null) {
                throw new IllegalStateException("Field layout and robot pose supplier are required");
            }
            
            return new VisionSubsystem(
                cameras,
                fieldLayout,
                simPoseSupplier,
                robotVelocitySupplier,
                usedPoseSupplier,
                config,
                poseConsumer
            );
        }
    }

    public static Builder builder() {
        return new Builder();
    }
}