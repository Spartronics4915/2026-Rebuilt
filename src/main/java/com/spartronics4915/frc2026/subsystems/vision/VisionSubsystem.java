package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.cameras.LimelightProcessor;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.filters.FilterInterface;
import com.spartronics4915.frc2026.subsystems.vision.filters.PipelineFilter;
import com.spartronics4915.frc2026.subsystems.vision.filters.ResultFilters;
import com.spartronics4915.frc2026.subsystems.vision.processing.PoseFusionEngine;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;

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
    private final PoseFusionEngine fusionEngine;

    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;

    private final PipelineFilter aprilTagFilter;
    private final VisionPoseConsumer poseConsumer;

    private final SwerveSubsystem swerve;

    private final List<ResultInterface> combinedResults = new ArrayList<>(4);
    private final List<ApriltagResult> combinedApriltagResults = new ArrayList<>(4);

    private final Pose3d[] tagPoseScratch = new Pose3d[33];

    private volatile boolean hasValidPose;

    private static final NetworkTable NT = NetworkTableInstance.getDefault().getTable("vision");

    private final StructPublisher<Pose2d> posePublisher = NT.getStructTopic("Vision Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> usedPosePublisher = NT.getStructTopic("Used Vision Pose", Pose2d.struct).publish();

    private final DoublePublisher transStdDevPublisher = NT.getDoubleTopic("XY Std Devs").publish();
    private final DoublePublisher rotStdDevPublisher = NT.getDoubleTopic("Theta Std Devs").publish();
    private final DoublePublisher avgAmbiguityPublisher = NT.getDoubleTopic("Avg Ambiguity").publish();
    private final DoublePublisher avgAreaPublisher = NT.getDoubleTopic("Avg Area").publish();
    private final DoublePublisher latencyPublisher = NT.getDoubleTopic("Latency").publish();
    private final DoublePublisher targetCountPublisher = NT.getDoubleTopic("Target Count").publish();

    private final StructArrayPublisher<Pose3d> trackedApriltagsPublisher =
        NT.getStructArrayTopic("Tracked Apriltags", Pose3d.struct).publish();

    public VisionSubsystem(
        Map<String, ProcessorInterface> cameras,
        AprilTagFieldLayout fieldLayout,
        VisionConfiguration configuration,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerveSubsystem
    ) {
        this.cameras = cameras;
        this.fusionEngine = new PoseFusionEngine();
        this.visionSystemSim = new VisionSystemSim("main");
        this.poseConsumer = poseConsumer;
        this.config = configuration;
        this.swerve = swerveSubsystem;
        this.hasValidPose = false;

        this.aprilTagFilter = new PipelineFilter(buildFilterList(configuration, swerveSubsystem));

        isSimulation = Robot.isSimulation();

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : cameras.values()) {
            camera.start();
        
            if (isSimulation) {
                Optional<PhotonCameraSim> cameraSim = camera.getCameraSim();
                cameraSim.ifPresent(sim ->
                    visionSystemSim.addCamera(sim, camera.getCameraTransform())
                );
            }
        }
    }

    private static List<FilterInterface> buildFilterList(VisionConfiguration config, SwerveSubsystem swerve) {
        List<FilterInterface> filters = new ArrayList<>();
        filters.add(new ResultFilters.LatencyFilter(config.maxLatencyMs));
        filters.add(new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore));
        filters.add(new ResultFilters.AreaFilter(config.minArea, config.maxArea));

        if (config.maxOdometryDeviationMeters < Double.MAX_VALUE) {
            filters.add(new ResultFilters.OdometryOutlierFilter(
                swerve::getPose, config.maxOdometryDeviationMeters));
        }

        return filters;
    }

    @Override
    public void periodic() {
        if (swerve != null) {
            double headingDegrees = swerve.getGyroRotation3d().toRotation2d().getDegrees();
            for (ProcessorInterface camera : cameras.values()) {
                if (camera instanceof LimelightProcessor ll) {
                    ll.updateHeading(headingDegrees);
                }
            }
        }

        collectResults();
        filterAndCollectApriltags();

        if (!combinedApriltagResults.isEmpty()) {
            processApriltags();
        } else {
            hasValidPose = false;
        }

        if (isSimulation) {
            visionSystemSim.update(swerve.getPose());
        }
    }

    private void collectResults() {
        combinedResults.clear();
        for (ProcessorInterface entry : cameras.values()) {
            entry.drainResultQueue(combinedResults);
        }
    }

    private void filterAndCollectApriltags() {
        combinedApriltagResults.clear();
        for (int i = 0; i < combinedResults.size(); i++) {
            ResultInterface result = combinedResults.get(i);
            if (result instanceof ApriltagResult ar
                    && ar.getStdDevs() != null
                    && aprilTagFilter.test(ar)
            ) {
                combinedApriltagResults.add(ar);
            }
        }
    }

    private void processApriltags() {
        Optional<ApriltagResult> fusedResultOpt = fusionEngine.fusePoses(combinedApriltagResults, config);
        if (fusedResultOpt.isEmpty()) {
            hasValidPose = false;
            return;
        }

        ApriltagResult fusedResult = fusedResultOpt.get();

        if (swerve != null && swerve.isFlatDebounced()) {
            poseConsumer.accept(
                fusedResult.getPose(),
                fusedResult.getTimestampSeconds(),
                fusedResult.getStdDevs()
            );
        }

        hasValidPose = true;
        publishPoseDiagnostics(fusedResult);
    }

    private void publishPoseDiagnostics(ApriltagResult fusedResult) {
        posePublisher.set(fusedResult.getPose());
        usedPosePublisher.set(swerve.getPastVisionPose(fusedResult.getTimestampSeconds()));

        transStdDevPublisher.set(fusedResult.getStdDevs().get(0, 0));
        rotStdDevPublisher.set(fusedResult.getStdDevs().get(2, 0));

        avgAmbiguityPublisher.set(fusedResult.getAmbiguity());
        avgAreaPublisher.set(fusedResult.getAverageArea());
        latencyPublisher.set(fusedResult.getLatencyMs());
        targetCountPublisher.set(fusedResult.getTargetCount());

        trackedApriltagsPublisher.accept(getTargetPoses(fusedResult.getTrackedTags()));
    }

    /**
     * Builds a {@link Pose3d} array for the given tags using the field layout.
     */
    public Pose3d[] getTargetPoses(List<TrackedTag> tags) {
        int count = 0;
        for (int i = 0; i < tags.size(); i++) {
            var tagPose = VisionConstants.apriltagFieldLayout.getTagPose(tags.get(i).fiducialId);
            if (tagPose.isPresent()) tagPoseScratch[count++] = tagPose.get();
        }

        Pose3d[] result = new Pose3d[count];
        System.arraycopy(tagPoseScratch, 0, result, 0, count);
        return result;
    }

    public boolean hasValidPose() { 
        return hasValidPose; 
    }

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
    }

}