package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.Constants.VisionConstants.FilterConstants;
import com.spartronics4915.frc2026.Constants.VisionConstants.FusionConstants;
import com.spartronics4915.frc2026.Constants.VisionConstants.VisionMeasurementMode;
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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Collects timestamped AprilTag measurements and sends them to the drivetrain estimator.
 * Each camera remains independent by default; optional fusion only combines simultaneous frames.
 */
public class VisionSubsystem extends SubsystemBase {
    private final List<ProcessorInterface> cameras;
    private final VisionPoseConsumer poseConsumer;
    private final SwerveSubsystem swerve;
    private final PipelineFilter resultFilter;
    private final PoseFusionEngine fusionEngine = new PoseFusionEngine();

    private final List<ResultInterface> rawResults = new ArrayList<>(16);
    private final List<ApriltagResult> acceptedResults = new ArrayList<>(16);
    private final List<ApriltagResult> fusionGroup = new ArrayList<>(8);
    private final Pose3d[] tagPoseScratch = new Pose3d[33];

    private volatile boolean hasValidPose;
    private Pose2d visionPose;

    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;

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
    private final BooleanPublisher fusedMeasurementsPublisher = table.getBooleanTopic("Fused Measurements").publish();

    public VisionSubsystem(
        AprilTagFieldLayout fieldLayout,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerve,
        List<ProcessorInterface> cameras
    ) {
        this.poseConsumer = poseConsumer;
        this.swerve = swerve;
        this.cameras = List.copyOf(cameras);
        this.resultFilter = buildFilter(swerve);
        this.isSimulation = Robot.isSimulation();
        this.visionSystemSim = isSimulation ? new VisionSystemSim("main") : null;

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : this.cameras) {
            startCamera(camera);
        }
    }

    private void startCamera(ProcessorInterface camera) {
        camera.start();
        if (isSimulation && visionSystemSim != null) {
            camera.getCameraSim().ifPresent(sim -> {
                visionSystemSim.addCamera(sim, camera.getCameraTransform());
                sim.enableDrawWireframe(false);
                sim.enableProcessedStream(false);
                sim.enableRawStream(false);
            });
        }
    }

    @Override
    public void periodic() {
        updateLimelightHeading();
        collectAcceptedResults();

        hasValidPose = !acceptedResults.isEmpty();
        hasValidPosePublisher.set(hasValidPose);
        fusedMeasurementsPublisher.set(VisionConstants.measurementMode == VisionMeasurementMode.FUSED);

        if (hasValidPose) {
            submitMeasurements();
            publishDiagnostics(acceptedResults.get(acceptedResults.size() - 1));
        }

        updateSimulation();
    }

    private void updateLimelightHeading() {
        if (swerve == null) return;
        double headingDegrees = swerve.getGyroRotation3d().toRotation2d().getDegrees();
        for (ProcessorInterface camera : cameras) {
            if (camera instanceof LimelightProcessor limelight) {
                limelight.setRobotHeading(headingDegrees);
            }
        }
    }

    private void collectAcceptedResults() {
        rawResults.clear();
        for (ProcessorInterface camera : cameras) {
            camera.drainResultQueue(rawResults);
        }

        acceptedResults.clear();
        for (ResultInterface result : rawResults) {
            if (result instanceof ApriltagResult tagResult
                && tagResult.getStdDevs() != null
                && resultFilter.test(tagResult)) {
                acceptedResults.add(tagResult);
            }
        }
        acceptedResults.sort(Comparator.comparingDouble(ApriltagResult::getTimestampSeconds));
    }

    private void submitMeasurements() {
        if (swerve == null || !swerve.isFlatDebounced()) return;

        if (VisionConstants.measurementMode == VisionMeasurementMode.INDIVIDUAL) {
            for (ApriltagResult result : acceptedResults) {
                submit(result);
            }
            return;
        }

        fusionGroup.clear();
        for (ApriltagResult result : acceptedResults) {
            if (!fusionGroup.isEmpty()
                && result.getTimestampSeconds() - fusionGroup.get(0).getTimestampSeconds()
                    > FusionConstants.timestampWindowSecs) {
                submitFusedGroup();
                fusionGroup.clear();
            }
            fusionGroup.add(result);
        }
        submitFusedGroup();
    }

    private void submitFusedGroup() {
        if (fusionGroup.isEmpty()) return;
        fusionEngine.fusePoses(fusionGroup).ifPresent(this::submit);
    }

    private void submit(ApriltagResult result) {
        poseConsumer.accept(result.getPose(), result.getTimestampSeconds(), result.getStdDevs());
    }

    private static PipelineFilter buildFilter(SwerveSubsystem swerve) {
        List<FilterInterface> filters = new ArrayList<>();
        filters.add(new ResultFilters.LatencyFilter(FilterConstants.maxLatencyMs));
        filters.add(new ResultFilters.AmbiguityFilter(FilterConstants.maxAmbiguity));
        filters.add(new ResultFilters.DistanceFilter(FilterConstants.maxSingleTagDistanceMeters));
        if (FilterConstants.maxOdometryDeviationMeters < Double.MAX_VALUE && swerve != null) {
            filters.add(new ResultFilters.OdometryOutlierFilter(
                swerve::getPose,
                FilterConstants.maxOdometryDeviationMeters
            ));
        }
        return new PipelineFilter(filters);
    }

    private void publishDiagnostics(ApriltagResult result) {
        visionPose = result.getPose();
        posePublisher.set(result.getPose());
        xyStdDevPublisher.set(result.getStdDevs().get(0, 0));
        thetaStdDevPublisher.set(result.getStdDevs().get(2, 0));
        ambiguityPublisher.set(result.getAmbiguity());
        areaPublisher.set(result.getAverageArea());
        latencyPublisher.set(result.getLatencyMs());
        targetCountPublisher.set(result.getTargetCount());
        trackedTagsPublisher.set(getTargetPoses(result.getTrackedTags()));
    }

    private void updateSimulation() {
        if (!isSimulation || visionSystemSim == null || swerve == null) return;
        for (ProcessorInterface camera : cameras) {
            camera.getCameraSim().ifPresent(sim -> visionSystemSim.adjustCamera(sim, camera.getCameraTransform()));
        }
        visionSystemSim.update(swerve.getPose());
    }

    public Pose3d[] getTargetPoses(List<TrackedTag> tags) {
        int count = 0;
        for (TrackedTag tag : tags) {
            Optional<Pose3d> pose = VisionConstants.apriltagFieldLayout.getTagPose(tag.getFiducialId());
            if (pose.isPresent() && count < tagPoseScratch.length) {
                tagPoseScratch[count++] = pose.get();
            }
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
