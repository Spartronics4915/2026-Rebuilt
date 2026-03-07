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
import com.spartronics4915.frc2026.subsystems.vision.filters.FilterInterface;
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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Manages all vision cameras, processes AprilTag detections, fuses pose
 * estimates across cameras, and submits measurements to the drivetrain
 * pose estimator.
 *
 * <p>Each camera runs its own {@link Notifier} thread via
 * {@link ProcessorInterface}, producing results asynchronously — including
 * their own standard deviations computed by a per-camera {@link StdDevCalculator}.
 * Every robot loop iteration, {@link #periodic()} drains those queues,
 * filters bad measurements, fuses across cameras via {@link PoseFusionEngine},
 * and forwards the final estimate to the drivetrain via {@link VisionPoseConsumer}.
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
    
    private final SwerveSubsystem swerve;

    private final List<ResultInterface> combinedResults = new ArrayList<>(16);
    private final List<ApriltagResult> combinedApriltagResults = new ArrayList<>(16);

    private volatile boolean hasValidPose;

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

        this.aprilTagFilter = new PipelineFilter(buildFilterList(configuration, swerveSubsystem));

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
     * Builds the ordered filter list from the given configuration.
     * Filters are ordered cheapest-first so expensive checks only run on
     * results that have already passed the early gates.
     */
    private static List<FilterInterface> buildFilterList(
        VisionConfiguration config,
        SwerveSubsystem swerve
    ) {
        // Build the filter pipeline in order of cheapest to most expensive filter,
        // so bad results are rejected early before more processing
        List<FilterInterface> filters = new ArrayList<>();
            filters.add(new ResultFilters.LatencyFilter(config.maxLatencyMs));
            filters.add(new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore));
            filters.add(new ResultFilters.DistanceFilter(config.maxSingleTagDistanceMeters, config.maxMultiTagDistanceMeters));
            filters.add(new ResultFilters.AnisotropyFilter(config.maxAnisotropy));
            filters.add(new ResultFilters.AreaFilter(config.minArea, config.maxArea));

        // OdometryOutlierFilter runs last, it requires an external supplier call :)
        if (config.maxOdometryDeviationMeters < Double.MAX_VALUE) {
            filters.add(new ResultFilters.OdometryOutlierFilter(
                swerve::getPose,
                config.maxOdometryDeviationMeters
            ));
        }

        return filters;
    }

    /**
     * Collects queued camera results, filters bad measurements, fuses across
     * cameras, and forwards the result to the drivetrain pose estimator
     */
    @Override
    public void periodic() {
        try {
            collectResults();
            filterAndCollectApriltags();

            // Fuse poses from multiple cameras
            if (!combinedApriltagResults.isEmpty()) {
                processApriltags();
            } else {
                // No results passed filtering this loop
                hasValidPose = false;
            }

            // Feed the ground truth robot pose back into the sim so virtual cameras
            // produce detections matching the robot's actual position
            if (isSimulation) {
                visionSystemSim.update(swerve.getRobotPose());
            }
        } finally {
            
        }
    }

    /**
     * Drains each camera's result queue
     */
    private void collectResults() {
        combinedResults.clear();
        for (ProcessorInterface entry : cameras.values()) {
            entry.drainResultQueue(combinedResults);
        }
    }

    /**
     * Removes results that fail quality thresholds, then narrows to ApriltagResult
     */
    private void filterAndCollectApriltags() {
        combinedApriltagResults.clear();
        for (ResultInterface result : combinedResults) {
            // Drop any results where std devs are missing (shouldn't happen)
            if (result instanceof ApriltagResult ar
                    && ar.getStdDevs() != null
                    && aprilTagFilter.test(ar)) {
                combinedApriltagResults.add(ar);
            }
        }
    }

    /**
     * Fuses scratchApriltags, submits the result to the drivetrain pose estimator,
     * and publishes all diagnostics to NetworkTables
     */
    private void processApriltags() {
        Optional<ApriltagResult> fusedResultOpt = PoseFusionEngine.fusePoses(combinedApriltagResults, config);
        if (fusedResultOpt.isEmpty()) {
            // Results exist but fusion produced no usable estimate
            hasValidPose = false;
            return;
        }

        ApriltagResult fusedResult = fusedResultOpt.get();

        // Only submit the pose if the robot is flat, tilt causes
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

        publishPoseDiagnostics(fusedResult);
        publishCameraPoses();

        //#endregion
    }

    /**
     * Publishes all pose quality and fusion diagnostics to NetworkTables.
     */
    private void publishPoseDiagnostics(ApriltagResult fusedResult) {
        posePublisher.set(fusedResult.getPose());
        usedPosePublisher.set(swerve.getPastVisionPose(fusedResult.getTimestampSeconds()));

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
    }

    /**
     * Publishes the current 3D world position of each physical camera.
     */
    private void publishCameraPoses() {
        rightCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.RIGHT_CAMERA_TRANSFORM));
        leftCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.LEFT_CAMERA_TRANSFORM));
        backCameraPosePublisher.accept(new Pose3d(swerve.getRobotPose()).plus(VisionConstants.CameraConstants.BACK_CAMERA_TRANSFORM));
    }

    /**
     * Returns a list of pose3ds from a list of {@link PhotonTrackedTarget}
     * 
     * @param targets a list of {@link PhotonTrackedTarget}
     * @return a list of all the pose3ds of the list of tags
     */
    public static Pose3d[] getTargetPoses(List<PhotonTrackedTarget> targets) {
        return targets.stream()
            .map(target -> VisionConstants.LAYOUT.getTagPose(target.fiducialId))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .toArray(Pose3d[]::new);
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