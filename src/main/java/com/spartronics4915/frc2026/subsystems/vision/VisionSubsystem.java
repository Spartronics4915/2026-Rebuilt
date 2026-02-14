package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.filters.PipelineFilter;
import com.spartronics4915.frc2026.subsystems.vision.filters.ResultFilters;
import com.spartronics4915.frc2026.subsystems.vision.processing.PoseFusionEngine;
import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.util.PerformanceTracker;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {

    private final Map<String, ProcessorInterface> cameras;
    private final VisionConfiguration config;
    
    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;
    
    private final PipelineFilter aprilTagFilter;
    private final StdDevCalculator stdDevCalculator;
    private final PoseFusionEngine fusionEngine;
    private final VisionPoseConsumer poseConsumer;
    
    private final PerformanceTracker performanceTracker;
    private final SwerveSubsystem swerve;

    private boolean hasValidPose;

    // Logging hell :(
    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> usedPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Used Vision Pose", Pose2d.struct).publish();

    private final StructPublisher<Pose3d> rightCameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Right Camera Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> leftCameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Left Camera Pose", Pose3d.struct).publish();

    private final DoublePublisher transStdDevPublisher = NetworkTableInstance.getDefault().getDoubleTopic("XY Std Devs").publish();
    private final DoublePublisher rotStdDevPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Theta Std Devs").publish();

    private final DoublePublisher avgDistancePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Avg Distance").publish();
    private final DoublePublisher avgAmbiguityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Avg Ambiguity").publish();
    private final DoublePublisher avgAreaPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Avg Area").publish();
    private final DoublePublisher xAnisotropyPublisher = NetworkTableInstance.getDefault().getDoubleTopic("X Anisotropy").publish();
    private final DoublePublisher yAnisotropyPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Y Anisotropy").publish();
    private final DoublePublisher latencyPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Latency").publish();
    private final DoublePublisher targetCountPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Target Count").publish();

    private final StructArrayPublisher<Pose3d> trackedApriltagsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Tracked Apriltags", Pose3d.struct).publish();
    
    public VisionSubsystem(
        Map<String, ProcessorInterface> cameras,
        AprilTagFieldLayout fieldLayout,
        VisionConfiguration configuration,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerveSubsystem
    ) {
        this.cameras = cameras;
        this.visionSystemSim = new VisionSystemSim("main");
        this.poseConsumer = poseConsumer;
        this.config = configuration;
        this.swerve = swerveSubsystem;
        this.hasValidPose = false;

        this.aprilTagFilter = new PipelineFilter(List.of(
            new ResultFilters.LatencyFilter(config.maxLatencyMs),
            new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore),
            new ResultFilters.DistanceFilter(config.maxSingleTagDistanceMeters, config.maxMultiTagDistanceMeters),
            new ResultFilters.AnisotropyFilter(config.maxAnisotropy),
            new ResultFilters.AreaFilter(config.minArea, config.maxArea)
        ));
        
        this.stdDevCalculator = new StdDevCalculator();
        this.fusionEngine = new PoseFusionEngine();
        this.performanceTracker = new PerformanceTracker(config.maxPeriodicTimeMs);

        isSimulation = Robot.isSimulation();
        
        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : cameras.values()) {
            camera.setSpeedSupplier(swerveSubsystem::getFieldVelocity);
            camera.start();
            if (isSimulation) visionSystemSim.addCamera(camera.getCameraSim(), camera.getCameraTransform());
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
        
        // Filter the camera results
        performanceTracker.startTiming("filtering");
        List<ResultInterface> filteredResults = aprilTagFilter.filter(allResults);
        performanceTracker.stopTiming();

        // Convert ResultInterface to ApriltagResult
        List<ApriltagResult> apriltagResults = filteredResults.stream()
            .filter(ApriltagResult.class::isInstance)
            .map(ApriltagResult.class::cast)
            .toList();

        // Set the standard deviations for the apriltag results
        for (ApriltagResult entry : apriltagResults) {
            entry.setStdDevs(stdDevCalculator.calculate(
                entry.getAverageDistanceToTargets(),
                entry.getAmbiguity(),
                entry.getAverageArea(),
                entry.getXAnisotropy(),
                entry.getYAnisotropy(),
                entry.getChassisSpeeds(),
                entry.getLatencyMs(), 
                entry.getTargetCount()
            ));
        }

        apriltagResults = apriltagResults.stream()
            .filter(result -> result.getStdDevs() != null)
            .toList();

        // Fuse poses from multiple cameras
        if (!apriltagResults.isEmpty()) {
            performanceTracker.startTiming("pose_fusion");
            
            ApriltagResult fusedResult = fusionEngine.fusePoses(apriltagResults, config);

            if (swerve != null && swerve.isFlatDebounced()) {
                poseConsumer.accept(
                    fusedResult.getPose(),
                    fusedResult.getTimestampSeconds(),
                    fusedResult.getStdDevs()
                );
            }
            
            performanceTracker.stopTiming();

            hasValidPose = true;

            // Logging hell (i'm sorry)
            posePublisher.set(fusedResult.getPose());
            usedPosePublisher.set(swerve.getPastVisionPose(fusedResult.getTimestampSeconds()));

            rightCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.RIGHT_CAMERA_TRANSFORM));
            leftCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.LEFT_CAMERA_TRANSFORM));

            transStdDevPublisher.set(fusedResult.getStdDevs().get(0, 0));
            rotStdDevPublisher.set(fusedResult.getStdDevs().get(2, 0));

            avgDistancePublisher .set(fusedResult.getAverageDistanceToTargets());
            avgAmbiguityPublisher.set(fusedResult.getAmbiguity());
            avgAreaPublisher.set(fusedResult.getAverageArea());
            xAnisotropyPublisher.set(fusedResult.getXAnisotropy());
            yAnisotropyPublisher.set(fusedResult.getYAnisotropy());
            latencyPublisher.set(fusedResult.getLatencyMs());
            targetCountPublisher.set(fusedResult.getTargets().size());
        } else {
            hasValidPose = false;
        }
        
        if (isSimulation) {
            visionSystemSim.update(swerve.getRobotPose());
        }
        
        // Stop periodic_total timing
        performanceTracker.stopTiming();
        
        // Publish performance metrics
        performanceTracker.publishMetrics();
    }

    public boolean hasValidPose() {
        return hasValidPose;
    }

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(
            Pose2d robotPose,
            double timestamp,
            Matrix<N3, N1> stdDevs
        );
    }

}
