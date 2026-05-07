package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.Constants.VisionConstants.FilterConstants;
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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// sim.enableDrawWireframe(false);
//                    sim.enableProcessedStream(false);
//                    sim.enableRawStream(false);
//                });
//            }
//        }
//
//        new Trigger(swerve::isFlatDebounced)
//            .onTrue(Commands.runOnce(() -> {
//                if (hasValidPose) {
//                    swerve.resetPose(fusedPose);
//                }
//            }).withName("Relocalize On Flat"));

/**
 * Manages all vision cameras and feeds fused pose estimates to the drive-train's
 * pose estimator.
 */
public class VisionSubsystem extends SubsystemBase {
    private final List<ProcessorInterface> primaryCameras;
    private final List<ProcessorInterface> fallbackCameras;
    private final List<ProcessorInterface> cameras;

    private final PoseFusionEngine fusionEngine;
    private final PipelineFilter resultFilter;

    private final List<ResultInterface> primaryRaw = new ArrayList<>(8);
    private final List<ApriltagResult> primaryFused = new ArrayList<>(8);

    private final List<ResultInterface> fallbackRaw = new ArrayList<>(8);
    private final List<ApriltagResult> fallbackFused = new ArrayList<>(8);

    private volatile boolean hasValidPose = false;
    private Pose2d visionPose;

    private final VisionPoseConsumer poseConsumer;
    private final SwerveSubsystem swerve;

    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;

    private final Pose3d[] tagPoseScratch = new Pose3d[33];

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");

    private final StructPublisher<Pose2d> posePublisher = table.getStructTopic("Vision Pose", Pose2d.struct).publish();

    private final DoublePublisher xyStdDevPublisher = table.getDoubleTopic("XY Std Devs").publish();
    private final DoublePublisher thetaStdDevPublisher = table.getDoubleTopic("Theta Std Devs").publish();
    private final DoublePublisher ambiguityPublisher = table.getDoubleTopic("Avg Ambiguity").publish();
    private final DoublePublisher areaPublisher = table.getDoubleTopic("Avg Area").publish();
    private final DoublePublisher latencyPublisher = table.getDoubleTopic("Latency").publish();
    private final DoublePublisher targetCountPublisher = table.getDoubleTopic("Target Count").publish();

    private final StructArrayPublisher<Pose3d> trackedTagsPublisher = table.getStructArrayTopic("Tracked Apriltags", Pose3d.struct).publish();
    private final BooleanPublisher hasValidPosePublisher = table.getBooleanTopic("Has Valid Pose").publish();
    private final BooleanPublisher currentPipelinePublisher = table.getBooleanTopic("Is Primary").publish();

    // Debug
    // private final StructPublisher<Pose3d> debugRobotPublisher = table.getStructTopic("Debug: Robot", Pose3d.struct).publish();
    // private final StructPublisher<Translation3d> debugTurretPublisher = table.getStructTopic("Debug: Turret", Translation3d.struct).publish();
    // private final StructPublisher<Transform3d> debugCameraPublisher = table.getStructTopic("Debug: Camera", Transform3d.struct).publish();

    public VisionSubsystem(
        AprilTagFieldLayout fieldLayout,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerve,
        List<ProcessorInterface> primaryCameras,
        List<ProcessorInterface> fallbackCameras
    ) {
        this.poseConsumer = poseConsumer;
        this.swerve = swerve;
        this.primaryCameras = List.copyOf(primaryCameras);
        this.fallbackCameras = List.copyOf(fallbackCameras);

        List<ProcessorInterface> combined = new ArrayList<>(primaryCameras);
        combined.addAll(fallbackCameras);

        this.cameras = List.copyOf(combined);

        this.fusionEngine = new PoseFusionEngine();
        this.resultFilter = buildFilter(swerve);

        this.isSimulation = Robot.isSimulation();
        this.visionSystemSim = (isSimulation) ? new VisionSystemSim("main") : null;

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : this.cameras) startCamera(camera);
    }

    public VisionSubsystem(
        AprilTagFieldLayout fieldLayout,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerve,
        List<ProcessorInterface> primaryCameras
    ) {
        this(fieldLayout, poseConsumer, swerve, primaryCameras, List.of());
    }

    private void startCamera(ProcessorInterface camera) {
        camera.start();
        if (isSimulation && visionSystemSim != null) {
            PhotonCameraSim sim = camera.getCameraSim().get();
                visionSystemSim.addCamera(sim, camera.getCameraTransform());
                sim.enableDrawWireframe(false);
                sim.enableProcessedStream(false);
                sim.enableRawStream(false);
        }
    }

    @Override
    public void periodic() {
        if (swerve != null) {
            double robotHeading = swerve.getGyroRotation3d().toRotation2d().getDegrees();
            cameras.forEach(camera -> {
                if (camera instanceof LimelightProcessor) camera.setRobotHeading(robotHeading);
            });
        }

        boolean primaryValid = processCameraPipeline(primaryCameras, resultFilter, primaryRaw, primaryFused);
        boolean fallbackValid = false;

        if (!primaryValid && !fallbackCameras.isEmpty()) {
            fallbackValid = processCameraPipeline(fallbackCameras, resultFilter, fallbackRaw, fallbackFused);
        }

        hasValidPose = primaryValid || fallbackValid;
        hasValidPosePublisher.accept(hasValidPose);
        currentPipelinePublisher.accept(primaryValid);

        if (primaryValid) publishDiagnostics(primaryFused);
        else if (fallbackValid) publishDiagnostics(fallbackFused);

        if (isSimulation && swerve != null && visionSystemSim != null) {
            visionSystemSim.update(swerve.getPose());
        }
    }

    private boolean processCameraPipeline(
        List<ProcessorInterface> cameras,
        PipelineFilter filter,
        List<ResultInterface> rawResults,
        List<ApriltagResult> collectedTagList
    ) {
        rawResults.clear();
        for (ProcessorInterface camera : cameras) camera.drainResultQueue(rawResults);

        collectedTagList.clear();
        for (ResultInterface result : rawResults) {
            if (result instanceof ApriltagResult tagResult 
                && tagResult.getStdDevs() != null 
                && filter.test(tagResult)) {
                collectedTagList.add(tagResult);
            }
        }

        if (collectedTagList.isEmpty()) return false;

        Optional<ApriltagResult> fused = fusionEngine.fusePoses(collectedTagList);
        if (fused.isEmpty()) return false;

        ApriltagResult result = fused.get();
        if (swerve != null && swerve.isFlatDebounced()) {
            poseConsumer.accept(
                result.getPose(),
                result.getTimestampSeconds(),
                result.getStdDevs()
            );
            return true;
        }
        return false;
    }

    @SuppressWarnings("unused")
    private static PipelineFilter buildFilter(SwerveSubsystem swerve) {
        List<FilterInterface> filter = new ArrayList<>();
            filter.add(new ResultFilters.LatencyFilter(FilterConstants.maxLatencyMs));
            filter.add(new ResultFilters.AmbiguityFilter(FilterConstants.maxAmbiguity));
            filter.add(new ResultFilters.AreaFilter(FilterConstants.minArea, FilterConstants.maxArea));
            filter.add(new ResultFilters.DistanceFilter(FilterConstants.maxSingleTagDistanceMeters));
        if (FilterConstants.maxOdometryDeviationMeters < Double.MAX_VALUE) {
            filter.add(
                new ResultFilters.OdometryOutlierFilter(
                    swerve::getPose,
                    FilterConstants.maxOdometryDeviationMeters
                )
            );
        }
        return new PipelineFilter(filter);
    }

    private void updateTagPublishScratch(List<TrackedTag> tags) {
        for (int i = 0; i < tags.size() && i < tagPoseScratch.length; i++) {
            Optional<Pose3d> pose = VisionConstants.apriltagFieldLayout.getTagPose(tags.get(i).getFiducialId());
            tagPoseScratch[i] = pose.orElse(new Pose3d());
        }

        for (int i = tags.size(); i < tagPoseScratch.length; i++) {
            tagPoseScratch[i] = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        }
    }

    private void publishDiagnostics(List<ApriltagResult> results) {
        if (results.isEmpty()) return;
        ApriltagResult latest = results.stream()
            .max((a, b) -> Double.compare(a.getTimestampSeconds(), b.getTimestampSeconds()))
            .orElse(results.get(0));

        updateTagPublishScratch(latest.getTrackedTags());

        visionPose = latest.getPose();
        posePublisher.set(latest.getPose());
        xyStdDevPublisher.set(latest.getStdDevs().get(0, 0));
        thetaStdDevPublisher.set(latest.getStdDevs().get(2, 0));
        ambiguityPublisher.set(latest.getAmbiguity());
        areaPublisher.accept(latest.getAverageArea());
        latencyPublisher.set(latest.getLatencyMs());
        targetCountPublisher.set(latest.getTargetCount());
        trackedTagsPublisher.set(tagPoseScratch);
    }

    public Pose3d[] getTargetPoses(List<TrackedTag> tags) {
        int count = 0;
        for (TrackedTag tag : tags) {
            Optional<Pose3d> pose = VisionConstants.apriltagFieldLayout.getTagPose(tag.getFiducialId());
            if (pose.isPresent()) tagPoseScratch[count++] = pose.get();
        }
        Pose3d[] result = new Pose3d[count];
        System.arraycopy(tagPoseScratch, 0, result, 0, count);
        return result;
    }

    public Pose2d getVisionPose() { 
        return hasValidPose ? visionPose : null; 
    }

    public List<ProcessorInterface> getCameras() {
        return cameras;
    }

    public boolean hasAnyPose() { 
        return hasValidPose; 
    }

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
    }
    
}
