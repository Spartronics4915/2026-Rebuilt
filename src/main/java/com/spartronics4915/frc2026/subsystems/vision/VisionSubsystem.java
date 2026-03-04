package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.filters.PipelineFilter;
import com.spartronics4915.frc2026.subsystems.vision.filters.ResultFilters;
import com.spartronics4915.frc2026.subsystems.vision.processing.PoseFusionEngine;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.util.general.PerformanceTracker;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
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
    private final VisionPoseConsumer poseConsumer;
    
    private final PerformanceTracker performanceTracker;
    private final SwerveSubsystem swerve;

    private boolean hasValidPose;

    private static final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");

    private final StructPublisher<Pose2d> posePublisher = visionTable.getStructTopic("Vision Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> usedPosePublisher = visionTable.getStructTopic("Used Vision Pose", Pose2d.struct).publish();

    private final StructPublisher<Pose3d> rightCameraPosePublisher = visionTable.getStructTopic("Right Camera Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> leftCameraPosePublisher = visionTable.getStructTopic("Left Camera Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> backCameraPosePublisher = visionTable.getStructTopic("Back Camera Pose", Pose3d.struct).publish();

    private final DoublePublisher transStdDevPublisher = visionTable.getDoubleTopic("XY Std Devs").publish();
    private final DoublePublisher rotStdDevPublisher = visionTable.getDoubleTopic("Theta Std Devs").publish();

    private final DoublePublisher avgDistancePublisher = visionTable.getDoubleTopic("Avg Distance").publish();
    private final DoublePublisher avgAmbiguityPublisher = visionTable.getDoubleTopic("Avg Ambiguity").publish();
    private final DoublePublisher avgAreaPublisher = visionTable.getDoubleTopic("Avg Area").publish();
    private final DoublePublisher xAnisotropyPublisher = visionTable.getDoubleTopic("X Anisotropy").publish();
    private final DoublePublisher yAnisotropyPublisher = visionTable.getDoubleTopic("Y Anisotropy").publish();
    private final DoublePublisher latencyPublisher = visionTable.getDoubleTopic("Latency").publish();
    private final DoublePublisher targetCountPublisher = visionTable.getDoubleTopic("Target Count").publish();

    private final StructArrayPublisher<Pose3d> trackedApriltagsPublisher = visionTable.getStructArrayTopic("Tracked Apriltags", Pose3d.struct).publish();
    
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
        try {
            List<ResultInterface> allResults = new ArrayList<>();
            for (ProcessorInterface entry : cameras.values()) {
                allResults.addAll(entry.getResultQueue());
            }

            List<ApriltagResult> apriltagResults = aprilTagFilter.filter(allResults).stream()
                .filter(ApriltagResult.class::isInstance)
                .map(ApriltagResult.class::cast)
                .toList();

            apriltagResults = apriltagResults.stream()
                .filter(result -> result.getStdDevs() != null)
                .toList();

            if (!apriltagResults.isEmpty()) {
                Optional<ApriltagResult> fusedResultOpt = PoseFusionEngine.fusePoses(apriltagResults, config);
                if (fusedResultOpt.isPresent()) {
                    ApriltagResult fusedResult = fusedResultOpt.get();
                    
                    if (swerve != null && swerve.isFlatDebounced()) {
                        poseConsumer.accept(
                            fusedResult.getPose(),
                            fusedResult.getTimestampSeconds(),
                            fusedResult.getStdDevs()
                        );
                    }
                    
                    hasValidPose = true;

                    posePublisher.set(fusedResult.getPose());
                    usedPosePublisher.set(swerve.getPastVisionPose(fusedResult.getTimestampSeconds()));

                    rightCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.RIGHT_CAMERA_TRANSFORM));
                    leftCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.LEFT_CAMERA_TRANSFORM));
                    backCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.BACK_CAMERA_TRANSFORM));

                    transStdDevPublisher.set(fusedResult.getStdDevs().get(0, 0));
                    rotStdDevPublisher.set(fusedResult.getStdDevs().get(2, 0));

                    avgDistancePublisher.set(fusedResult.getAverageDistanceToTargets());
                    avgAmbiguityPublisher.set(fusedResult.getAmbiguity());
                    avgAreaPublisher.set(fusedResult.getAverageArea());
                    xAnisotropyPublisher.set(fusedResult.getXAnisotropy());
                    yAnisotropyPublisher.set(fusedResult.getYAnisotropy());
                    latencyPublisher.set(fusedResult.getLatencyMs());
                    targetCountPublisher.set(fusedResult.getTargets().size());

                    trackedApriltagsPublisher.accept(getTargetPoses(fusedResult.getTargets()));
                } else {
                    hasValidPose = false;
                }
            } else {
                hasValidPose = false;
            }

            if (isSimulation) {
                visionSystemSim.update(swerve.getRobotPose());
            }
        } finally {
            // Always stop timing exactly once — even if an exception is thrown above.
            performanceTracker.stopTiming();
            performanceTracker.publishMetrics();
        }
    }

    public static Pose3d[] getTargetPoses(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .map(target -> VisionConstants.LAYOUT.getTagPose(target.fiducialId))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toArray(Pose3d[]::new);
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