package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manages all vision cameras, processes AprilTag detections, fuses pose
 * estimates across cameras, and submits measurements to the drivetrain
 * pose estimator.
 *
 * <p>Each camera runs its own {@link Notifier} thread via
 * {@link ProcessorInterface}, producing results asynchronously.
 * Every robot loop iteration, {@link #periodic()} drains those queues,
 * filters bad measurements, calculates per-result standard deviations,
 * fuses across cameras via {@link PoseFusionEngine}, and forwards the
 * final estimate to the drivetrain via {@link VisionPoseConsumer}.
 *
 * <p>In simulation, a {@link VisionSystemSim} mirrors the real camera
 * pipeline using the robot's ground-truth pose.
 */
public class VisionSubsystem extends SubsystemBase {

    private final Map<String, ProcessorInterface> cameras;
    private final VisionConfiguration config;
    
    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;
    
    private final PipelineFilter aprilTagFilter;
    private final VisionPoseConsumer poseConsumer;
    
    //private final PerformanceTracker performanceTracker;
    private final SwerveSubsystem swerve;

    // True if the most recent periodic loop produced a valid pose
    private boolean hasValidPose;

    // Cached NetworkTable reference to avoid repeated map lookups each loop
    private static final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("vision");

    // -- Vision Poses --
    private final StructPublisher<Pose2d> posePublisher = visionTable.getStructTopic("Vision Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> usedPosePublisher = visionTable.getStructTopic("Used Vision Pose", Pose2d.struct).publish();

    // -- Camera Visualization --
    private final StructPublisher<Pose3d> rightCameraPosePublisher = visionTable.getStructTopic("Right Camera Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> leftCameraPosePublisher = visionTable.getStructTopic("Left Camera Pose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> backCameraPosePublisher = visionTable.getStructTopic("Back Camera Pose", Pose3d.struct).publish();

    // -- Standard Deviations --
    private final DoublePublisher transStdDevPublisher = visionTable.getDoubleTopic("XY Std Devs").publish();
    private final DoublePublisher rotStdDevPublisher = visionTable.getDoubleTopic("Theta Std Devs").publish();

    // -- Result Info --
    private final DoublePublisher avgDistancePublisher = visionTable.getDoubleTopic("Avg Distance").publish();
    private final DoublePublisher avgAmbiguityPublisher = visionTable.getDoubleTopic("Avg Ambiguity").publish();
    private final DoublePublisher avgAreaPublisher = visionTable.getDoubleTopic("Avg Area").publish();
    private final DoublePublisher xAnisotropyPublisher = visionTable.getDoubleTopic("X Anisotropy").publish();
    private final DoublePublisher yAnisotropyPublisher = visionTable.getDoubleTopic("Y Anisotropy").publish();
    private final DoublePublisher latencyPublisher = visionTable.getDoubleTopic("Latency").publish();
    private final DoublePublisher targetCountPublisher = visionTable.getDoubleTopic("Target Count").publish();

    // -- Tags Used --
    private final StructArrayPublisher<Pose3d> trackedApriltagsPublisher = visionTable.getStructArrayTopic("Tracked Apriltags", Pose3d.struct).publish();
    
    /**
     * Constructs the VisionSubsystem and starts all camera processing threads.
     *
     * @param cameras map of camera name to {@link ProcessorInterface}, one entry per physical camera
     * @param fieldLayout AprilTag field layout used for pose estimation
     * @param configuration tuning parameters for filtering, fusion, and std devs
     * @param poseConsumer callback that forwards accepted pose estimates to the drivetrain's pose estimator
     * @param swerveSubsystem drivetrain reference used for robot velocity and tilt debouncing
     */
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

        // Build the filter pipeline in order of cheapest to most expensive filter,
        // so bad results are rejected early before more processing
        this.aprilTagFilter = new PipelineFilter(List.of(
            new ResultFilters.LatencyFilter(config.maxLatencyMs),
            new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore),
            new ResultFilters.DistanceFilter(config.maxSingleTagDistanceMeters, config.maxMultiTagDistanceMeters),
            new ResultFilters.AnisotropyFilter(config.maxAnisotropy),
            new ResultFilters.AreaFilter(config.minArea, config.maxArea)
        ));
        
        //this.performanceTracker = new PerformanceTracker(config.maxPeriodicTimeMs);

        isSimulation = Robot.isSimulation();
        
        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            // Disable version check so sim cameras don't fail on version mismatch
            PhotonCamera.setVersionCheckEnabled(false);
        }

        // Wire each camera's velocity supplier and start its processing thread
        for (ProcessorInterface camera : cameras.values()) {
            camera.setSpeedSupplier(swerveSubsystem::getFieldVelocity);
            camera.start();
            if (isSimulation) visionSystemSim.addCamera(camera.getCameraSim(), camera.getCameraTransform());
        }
    }
    
    /**
     * Collects queued camera results, filters bad measurements, 
     * computes standard deviations, fuses across cameras, and
     * forwards the result to the drivetrain pose estimator.
     *
     * <p>Performance of each stage is tracked via {@link PerformanceTracker}
     * and published to NetworkTables for tuning and diagnostics.
     */
    @Override
    public void periodic() {
        //performanceTracker.startTiming("periodic_total");
        
        // Drain each camera's result queue. Camera threads write asynchronously
        // via Notifier, so getResultQueue() is a destructive thread-safe read.
        List<ResultInterface> allResults = new ArrayList<>();
        for (ProcessorInterface entry : cameras.values()) {
            //performanceTracker.startTiming("camera_" + entry.getCameraName());
            allResults.addAll(entry.getResultQueue());
            //performanceTracker.stopTiming();
        }
        
        // Remove results that fail quality thresholds, then narrow the type to
        // ApriltagResult since that's all this currently handles
        //performanceTracker.startTiming("filtering");
        List<ApriltagResult> apriltagResults = aprilTagFilter.filter(allResults).stream()
            .filter(ApriltagResult.class::isInstance)
            .map(ApriltagResult.class::cast)
            .toList();
        //performanceTracker.stopTiming();

        // Each result gets its own std devs based on distance, ambiguity, area,
        // anisotropy, motion, latency, and tag count. These are used both for
        // outlier rejection in fusion and for weighting in the pose estimator.
        for (ApriltagResult entry : apriltagResults) {
            entry.setStdDevs(StdDevCalculator.calculate(
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

        // Drop any results where std dev calculation returned null
        apriltagResults = apriltagResults.stream()
            .filter(result -> result.getStdDevs() != null)
            .toList();

        // Fuse poses from multiple cameras
        if (!apriltagResults.isEmpty()) {
            //performanceTracker.startTiming("pose_fusion");
            try {
                Optional<ApriltagResult> fusedResultOpt = PoseFusionEngine.fusePoses(apriltagResults, config);
                if (fusedResultOpt.isPresent()) {
                    ApriltagResult fusedResult = fusedResultOpt.get();
                    
                    // Only submit the pose if the robot is flat — tilt causes
                    // the 2D projection from 3D tag poses to be inaccurate.
                    if (swerve != null && swerve.isFlatDebounced()) {
                        poseConsumer.accept(
                            fusedResult.getPose(),
                            fusedResult.getTimestampSeconds(),
                            fusedResult.getStdDevs()
                        );
                    }
                
                    hasValidPose = true;
                
                    //#region Logging :)

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

                    //#endregion
                } else {
                    // Results exist but fusion produced no usable estimate
                    hasValidPose = false;
                }
            } finally {
                //performanceTracker.stopTiming();
            }
        } else {
            // No results passed filtering this loop
            hasValidPose = false;
        }
        
        // Feed the ground-truth robot pose back into the sim so virtual cameras
        // produce detections matching the robot's actual position
        if (isSimulation) {
            visionSystemSim.update(swerve.getRobotPose());
        }

        //performanceTracker.stopTiming();
        //performanceTracker.publishMetrics();
    }

    /**
     * Returns whether the most recent periodic loop produced a valid fused pose
     *
     * @return true if a valid pose was accepted this loop
     */
    public boolean hasValidPose() {
        return hasValidPose;
    }

    /**
     * Callback for submitting a vision pose estimate to the drivetrain's pose estimator.
     * Accepts the estimated pose, the timestamp when the image was captured, and the
     * measurement standard deviations representing confidence.
     */
    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(
            Pose2d robotPose,
            double timestamp,
            Matrix<N3, N1> stdDevs
        );
    }

}
