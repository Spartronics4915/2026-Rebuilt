package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.subsystems.vision.cameras.CameraInterface;
import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext.VisionState;
import com.spartronics4915.frc2026.subsystems.vision.filters.PipelineFilter;
import com.spartronics4915.frc2026.subsystems.vision.filters.ResultFilters;
import com.spartronics4915.frc2026.subsystems.vision.strategies.AprilTagStrategy;
import com.spartronics4915.frc2026.subsystems.vision.strategies.PipelineStrategyInterface;
import com.spartronics4915.frc2026.subsystems.vision.strategies.PipelineStrategyInterface.PoseEstimate;
import com.spartronics4915.frc2026.subsystems.vision.strategies.PipelineStrategyInterface.StrategyResult;
import com.spartronics4915.frc2026.util.PerformanceTracker;
import com.spartronics4915.frc2026.util.PoseFusionEngine;
import com.spartronics4915.frc2026.util.StdDevCalculator;

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
    
    private final Map<String, CameraInterface> cameras;
    private final Map<String, PipelineStrategyInterface> strategies;
    private final VisionContext context;
    private final VisionConfiguration config;
    private final VisionSystemSim visionSystemSim;
    
    private final PipelineFilter aprilTagFilter;
    private final PipelineFilter objectFilter;
    
    private final PoseFusionEngine fusionEngine;
    private final StdDevCalculator stdDevCalculator;
    private final PerformanceTracker performanceTracker;
    
    private final VisionPoseConsumer poseConsumer;
    private final VisionObjectsConsumer objectsConsumer;
    
    private Supplier<ChassisSpeeds> robotVelocitySupplier;
    private ChassisSpeeds lastVelocity;

    private static double lastPoseTimestamp;

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Pose2d> fusedPoseSupplier;

    private StructPublisher<Pose2d> rawVisionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Raw Vision Pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> fusedVisionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Fused Vision Pose", Pose2d.struct).publish();

    private StructPublisher<Pose3d> rightCameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Right Camera Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> leftCameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Left Camera Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> backCameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Back Camera Pose", Pose3d.struct).publish();

    private DoublePublisher translationStdDevPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Translation Std Devs").publish();
    private DoublePublisher rotationStdDevPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Rotation Std Devs").publish();

    private VisionSubsystem(
        Map<String, CameraInterface> cameras,
        AprilTagFieldLayout fieldLayout,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<ChassisSpeeds> robotVelocitySupplier,
        Supplier<Pose2d> fusedPoseSupplier,
        VisionConfiguration config,
        VisionPoseConsumer poseConsumer,
        VisionObjectsConsumer objectsConsumer
    ) {
        this.cameras = cameras;
        this.config = config;
        this.visionSystemSim = new VisionSystemSim("main");
        this.poseConsumer = poseConsumer;
        this.objectsConsumer = objectsConsumer;
        this.robotVelocitySupplier = robotVelocitySupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.fusedPoseSupplier = fusedPoseSupplier;
        this.lastVelocity = new ChassisSpeeds();
        
        this.context = new VisionContext(
            fieldLayout,
            robotPoseSupplier,
            config,
            VisionState.GLOBAL
        );
        
        this.strategies = new HashMap<>();
        this.strategies.put("apriltag", new AprilTagStrategy());

        this.aprilTagFilter = PipelineFilter.builder()
            .addFilter(ResultFilters.HAS_TARGETS)
            .addFilter(ResultFilters.HAS_POSE)
            .addFilter(new ResultFilters.LatencyFilter(config.maxLatencyMs))
            .addFilter(new ResultFilters.DistanceFilter(
                config.maxSingleTagDistanceMeters,
                config.maxMultiTagDistanceMeters
            ))
            .addFilter(new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore))
            .build();
        
        this.objectFilter = PipelineFilter.builder()
            .addFilter(ResultFilters.HAS_TARGETS)
            .addFilter(new ResultFilters.LatencyFilter(config.maxLatencyMs))
            .build();
        
        this.fusionEngine = new PoseFusionEngine();
        this.stdDevCalculator = new StdDevCalculator();
        this.performanceTracker = new PerformanceTracker(config.maxPeriodicTimeMs);

        visionSystemSim.addAprilTags(fieldLayout);

        for (CameraInterface camera : cameras.values()) {
            camera.start();
            visionSystemSim.addCamera(camera.getCameraSim(), camera.getTransform());
        }
    }

    @Override
    public void periodic() {
        performanceTracker.startTiming("periodic_total");
        
        if (robotVelocitySupplier != null) lastVelocity = robotVelocitySupplier.get();
        
        List<CameraResult> allResults = new ArrayList<>();
        
        for (Map.Entry<String, CameraInterface> entry : cameras.entrySet()) {
            String cameraName = entry.getKey();
            CameraInterface camera = entry.getValue();
            
            performanceTracker.startTiming("camera_" + cameraName);
            
            List<CameraResult> cameraResults = camera.processFrame(context);
            allResults.addAll(cameraResults);
            
            performanceTracker.stopTiming();
        }
        
        performanceTracker.startTiming("filtering");
        List<CameraResult> filteredResults = aprilTagFilter.filter(allResults);
        performanceTracker.stopTiming();
        
        performanceTracker.startTiming("strategy_apriltag");
        PipelineStrategyInterface strategy = strategies.get("apriltag");
        StrategyResult result = strategy.process(filteredResults, context);
        performanceTracker.stopTiming();

        if (result.isSuccessful() && !result.getPoseEstimates().isEmpty()) {
            performanceTracker.startTiming("pose_fusion");
            
            List<PoseEstimate> estimates = result.getPoseEstimates();
            
            if (config.enableMotionPunishment && robotVelocitySupplier != null) {
                estimates = applyMotionPunishment(estimates);
            }

            PoseEstimate fusedPose = fusionEngine.fusePoses(estimates, config);
            lastPoseTimestamp = fusedPose.getTimestamp();
            
            if (fusedPose != null) {
                poseConsumer.accept(
                    fusedPose.getPose(),
                    fusedPose.getTimestamp(),
                    fusedPose.getStdDevs()
                );
                translationStdDevPublisher.accept(fusedPose.getStdDevs().get(0, 0));
                rotationStdDevPublisher.accept(fusedPose.getStdDevs().get(2, 0));
                rawVisionPosePublisher.accept(fusedPose.getPose());
            }
            
            performanceTracker.stopTiming();
        }

        visionSystemSim.update(robotPoseSupplier.get());
        
        fusedVisionPosePublisher.accept(fusedPoseSupplier.get());

        rightCameraPosePublisher.accept(new Pose3d(robotPoseSupplier.get()).plus(VisionConstants.RIGHT_CAMERA_TRANSFORM));
        leftCameraPosePublisher.accept(new Pose3d(robotPoseSupplier.get()).plus(VisionConstants.LEFT_CAMERA_TRANSFORM));
        backCameraPosePublisher.accept(new Pose3d(robotPoseSupplier.get()).plus(VisionConstants.BACK_CAMERA_TRANSFORM));
        
        performanceTracker.stopTiming();
        performanceTracker.publishMetrics();
    }

    private List<PoseEstimate> applyMotionPunishment(List<PoseEstimate> estimates) {
        double linearVelocity = Math.sqrt(
            lastVelocity.vxMetersPerSecond * lastVelocity.vxMetersPerSecond +
            lastVelocity.vyMetersPerSecond * lastVelocity.vyMetersPerSecond
        );
        double angularVelocity = Math.abs(lastVelocity.omegaRadiansPerSecond);
        
        List<PoseEstimate> adjusted = new ArrayList<>(estimates.size());
        
        for (PoseEstimate estimate : estimates) {
            Matrix<N3, N1> adjustedStdDevs = stdDevCalculator.applyMotionPunishment(
                estimate.getStdDevs(),
                linearVelocity,
                angularVelocity,
                config
            );
            
            adjusted.add(new PoseEstimate(
                estimate.getPose(),
                estimate.getTimestamp(),
                adjustedStdDevs,
                estimate.getSource() + "-punished"
            ));
        }
        
        return adjusted;
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
        CameraInterface processor = cameras.get(cameraName);
        return processor != null && processor.isConnected();
    }

    public int getActiveCameraCount() {
        return (int) cameras.values().stream()
            .filter(CameraInterface::isConnected)
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
        private final Map<String, CameraInterface> cameras = new HashMap<>();
        private AprilTagFieldLayout fieldLayout;
        private Supplier<Pose2d> robotPoseSupplier;
        private Supplier<ChassisSpeeds> robotVelocitySupplier;
        private Supplier<Pose2d> fusedPoseSupplier;
        private VisionConfiguration config = new VisionConfiguration();
        private VisionPoseConsumer poseConsumer = (pose, time, std) -> {};
        private VisionObjectsConsumer objectsConsumer = (pose, time) -> {};

        public Builder addCamera(String name, CameraInterface processor) {
            cameras.put(name, processor);
            return this;
        }

        public Builder setFieldLayout(AprilTagFieldLayout layout) {
            this.fieldLayout = layout;
            return this;
        }

        public Builder setRobotPoseSupplier(Supplier<Pose2d> supplier) {
            this.robotPoseSupplier = supplier;
            return this;
        }

        public Builder setRobotVelocitySupplier(Supplier<ChassisSpeeds> supplier) {
            this.robotVelocitySupplier = supplier;
            return this;
        }

        public Builder setFusedPoseSupplier(Supplier<Pose2d> supplier) {
            this.fusedPoseSupplier = supplier;
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

        public Builder setObjectsConsumer(VisionObjectsConsumer consumer) {
            this.objectsConsumer = consumer;
            return this;
        }

        public VisionSubsystem build() {
            if (fieldLayout == null || robotPoseSupplier == null) {
                throw new IllegalStateException("Field layout and robot pose supplier are required");
            }
            
            return new VisionSubsystem(
                cameras,
                fieldLayout,
                robotPoseSupplier,
                robotVelocitySupplier,
                fusedPoseSupplier,
                config,
                poseConsumer,
                objectsConsumer
            );
        }
    }

    public static Builder builder() {
        return new Builder();
    }
}
