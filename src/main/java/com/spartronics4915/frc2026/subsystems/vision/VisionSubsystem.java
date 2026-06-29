package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

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
 * Manages all vision cameras and feeds fused pose estimates to the drivetrain's
 * pose estimator.
 *
 * <p><b>Allocation budget on the main thread (periodic path):</b>
 * <ul>
 *   <li>Zero {@code Optional} allocations — {@link PoseFusionEngine} uses a sentinel
 *       boolean {@code hasFusedResult()} instead.
 *   <li>Zero per-cycle tag-pose allocations — tag poses are resolved once at startup
 *       into a flat {@code Pose3d[]} lookup table indexed by fiducial ID.
 *   <li>One {@code Pose2d} + one {@code Rotation2d} per cycle when multi-camera fusion
 *       is active (unavoidable — {@link ApriltagResult#set} takes a {@code Pose2d}).
 *       Single-camera and best-result paths are fully zero-allocation.
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

    // ── Camera lists ──────────────────────────────────────────────────────────
    private final List<ProcessorInterface> primaryCameras;
    private final List<ProcessorInterface> fallbackCameras;
    private final List<ProcessorInterface> cameras;

    // ── Vision pipeline ───────────────────────────────────────────────────────
    private final PoseFusionEngine fusionEngine;
    private final PipelineFilter   resultFilter;

    // ── Per-pipeline scratch lists (reused every periodic call) ───────────────
    private final List<ResultInterface>  primaryRaw    = new ArrayList<>(8);
    private final List<ApriltagResult>   primaryFused  = new ArrayList<>(8);
    private final List<ResultInterface>  fallbackRaw   = new ArrayList<>(8);
    private final List<ApriltagResult>   fallbackFused = new ArrayList<>(8);

    // ── State ─────────────────────────────────────────────────────────────────
    private volatile boolean hasValidPose = false;
    private Pose2d visionPose;

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final VisionPoseConsumer poseConsumer;
    private final SwerveSubsystem    swerve;

    // ── Simulation ────────────────────────────────────────────────────────────
    private final VisionSystemSim visionSystemSim;
    private final boolean         isSimulation;

    // ── Tag-pose lookup table (zero-allocation alternative to getTagPose()) ───
    // Index = fiducial ID (1-based); index 0 is unused. Null entries = unknown tag.
    private static final int TAG_LOOKUP_SIZE = 34; // covers IDs 0-33
    private final Pose3d[] tagPoseLookup = new Pose3d[TAG_LOOKUP_SIZE];

    // ── Tag publish scratch (reused; only reallocates when visible-tag count changes) ──
    private final Pose3d[] tagPoseScratch    = new Pose3d[33];
    private int            tagPublishCount   = 0;
    private int            lastTagPublishCount = -1;
    private Pose3d[]       tagPublishBuffer  = new Pose3d[0];

    // ── NetworkTables publishers ──────────────────────────────────────────────
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("vision");

    private final StructPublisher<Pose2d>    posePublisher        = table.getStructTopic("Vision Pose",       Pose2d.struct).publish();
    private final DoublePublisher            xyStdDevPublisher    = table.getDoubleTopic("XY Std Devs").publish();
    private final DoublePublisher            thetaStdDevPublisher = table.getDoubleTopic("Theta Std Devs").publish();
    private final DoublePublisher            ambiguityPublisher   = table.getDoubleTopic("Avg Ambiguity").publish();
    private final DoublePublisher            areaPublisher        = table.getDoubleTopic("Avg Area").publish();
    private final DoublePublisher            latencyPublisher     = table.getDoubleTopic("Latency").publish();
    private final DoublePublisher            targetCountPublisher = table.getDoubleTopic("Target Count").publish();
    private final StructArrayPublisher<Pose3d> trackedTagsPublisher = table.getStructArrayTopic("Tracked Apriltags", Pose3d.struct).publish();
    private final BooleanPublisher           hasValidPosePublisher= table.getBooleanTopic("Has Valid Pose").publish();
    private final BooleanPublisher           currentPipelinePublisher = table.getBooleanTopic("Is Primary").publish();

    // ─────────────────────────────────────────────────────────────────────────

    public VisionSubsystem(
        AprilTagFieldLayout fieldLayout,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerve,
        List<ProcessorInterface> primaryCameras,
        List<ProcessorInterface> fallbackCameras
    ) {
        this.poseConsumer     = poseConsumer;
        this.swerve           = swerve;
        this.primaryCameras   = List.copyOf(primaryCameras);
        this.fallbackCameras  = List.copyOf(fallbackCameras);

        List<ProcessorInterface> combined = new ArrayList<>(primaryCameras);
        combined.addAll(fallbackCameras);
        this.cameras = List.copyOf(combined);

        this.fusionEngine = new PoseFusionEngine();
        this.resultFilter = buildFilter(swerve);

        this.isSimulation   = Robot.isSimulation();
        this.visionSystemSim = isSimulation ? new VisionSystemSim("main") : null;

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        // Build zero-allocation tag-pose lookup table once at startup.
        for (edu.wpi.first.apriltag.AprilTag tag : fieldLayout.getTags()) {
            int id = tag.ID;
            if (id >= 0 && id < TAG_LOOKUP_SIZE) {
                tagPoseLookup[id] = tag.pose;
            }
        }

        // Pre-fill scratch with kZero so it is always safe to publish.
        for (int i = 0; i < tagPoseScratch.length; i++) tagPoseScratch[i] = Pose3d.kZero;

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

    // ── Lifecycle ─────────────────────────────────────────────────────────────

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

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        if (swerve != null) {
            double robotHeading = swerve.getGyroRotation3d().toRotation2d().getDegrees();
            // Explicit for-loop avoids forEach lambda overhead on some JVMs.
            for (int i = 0; i < cameras.size(); i++) {
                ProcessorInterface camera = cameras.get(i);
                if (camera instanceof LimelightProcessor) camera.setRobotHeading(robotHeading);
            }
        }

        boolean primaryValid  = processCameraPipeline(primaryCameras,  resultFilter, primaryRaw,  primaryFused);
        boolean fallbackValid = false;

        if (!primaryValid && !fallbackCameras.isEmpty()) {
            fallbackValid = processCameraPipeline(fallbackCameras, resultFilter, fallbackRaw, fallbackFused);
        }

        hasValidPose = primaryValid || fallbackValid;
        hasValidPosePublisher.accept(hasValidPose);
        currentPipelinePublisher.accept(primaryValid);

        if (primaryValid)      publishDiagnostics(primaryFused);
        else if (fallbackValid) publishDiagnostics(fallbackFused);

        if (isSimulation && swerve != null && visionSystemSim != null) {
            visionSystemSim.update(swerve.getPose());
        }
    }

    // ── Pipeline ──────────────────────────────────────────────────────────────

    private boolean processCameraPipeline(
        List<ProcessorInterface> cams,
        PipelineFilter filter,
        List<ResultInterface> rawResults,
        List<ApriltagResult> collectedTagList
    ) {
        rawResults.clear();
        for (int i = 0; i < cams.size(); i++) cams.get(i).drainResultQueue(rawResults);

        collectedTagList.clear();
        for (int i = 0; i < rawResults.size(); i++) {
            ResultInterface result = rawResults.get(i);
            if (result instanceof ApriltagResult tagResult
                && tagResult.getStdDevs() != null
                && filter.test(tagResult)) {
                collectedTagList.add(tagResult);
            }
        }

        if (collectedTagList.isEmpty()) return false;

        // fusePoses() writes into fusionEngine.getFusedResult() — no Optional allocation.
        fusionEngine.fusePoses(collectedTagList);
        if (!fusionEngine.hasFusedResult()) return false;

        ApriltagResult result = fusionEngine.getFusedResult();
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

    // ── Diagnostics ───────────────────────────────────────────────────────────

    private void publishDiagnostics(List<ApriltagResult> results) {
        if (results.isEmpty()) return;

        // Find latest result without stream allocation.
        ApriltagResult latest = results.get(0);
        for (int i = 1; i < results.size(); i++) {
            if (results.get(i).getTimestampSeconds() > latest.getTimestampSeconds()) {
                latest = results.get(i);
            }
        }

        List<TrackedTag> trackedTags = latest.getTrackedTags();
        updateTagPublishScratch(trackedTags);

        visionPose = latest.getPose();
        posePublisher.set(latest.getPose());
        xyStdDevPublisher.set(latest.getStdDevs().get(0, 0));
        thetaStdDevPublisher.set(latest.getStdDevs().get(2, 0));
        ambiguityPublisher.set(latest.getAmbiguity());
        areaPublisher.accept(latest.getAverageArea());
        latencyPublisher.set(latest.getLatencyMs());
        targetCountPublisher.set(latest.getTargetCount());

        // Reallocate publish buffer only when the visible-tag count changes (rare).
        if (tagPublishCount != lastTagPublishCount) {
            tagPublishBuffer    = new Pose3d[tagPublishCount];
            lastTagPublishCount = tagPublishCount;
        }
        System.arraycopy(tagPoseScratch, 0, tagPublishBuffer, 0, tagPublishCount);
        trackedTagsPublisher.set(tagPublishBuffer);
    }

    /**
     * Fills {@link #tagPoseScratch} using the pre-built {@link #tagPoseLookup} table.
     * Zero Optional allocations — direct array index lookup.
     */
    private void updateTagPublishScratch(List<TrackedTag> tags) {
        tagPublishCount = 0;
        for (int i = 0; i < tags.size() && i < tagPoseScratch.length; i++) {
            int id = tags.get(i).getFiducialId();
            Pose3d pose = (id >= 0 && id < TAG_LOOKUP_SIZE) ? tagPoseLookup[id] : null;
            tagPoseScratch[tagPublishCount++] = (pose != null) ? pose : Pose3d.kZero;
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Returns the array of field-frame tag poses visible in the given tracked-tag list.
     * Uses the pre-built lookup table — zero Optional allocation.
     *
     * @param tags the tracked tags to resolve
     * @return a freshly allocated array containing only the tags with known poses
     */
    public Pose3d[] getTargetPoses(List<TrackedTag> tags) {
        int count = 0;
        for (int i = 0; i < tags.size() && count < tagPoseScratch.length; i++) {
            int id = tags.get(i).getFiducialId();
            if (id >= 0 && id < TAG_LOOKUP_SIZE && tagPoseLookup[id] != null) {
                tagPoseScratch[count++] = tagPoseLookup[id];
            }
        }
        Pose3d[] result = new Pose3d[count];
        System.arraycopy(tagPoseScratch, 0, result, 0, count);
        return result;
    }

    public Pose2d getVisionPose()         { return hasValidPose ? visionPose : null; }
    public List<ProcessorInterface> getCameras() { return cameras; }
    public boolean hasAnyPose()           { return hasValidPose; }

    // ── Filter builder ────────────────────────────────────────────────────────

    @SuppressWarnings("unused")
    private static PipelineFilter buildFilter(SwerveSubsystem swerve) {
        List<FilterInterface> filter = new ArrayList<>();
        filter.add(new ResultFilters.LatencyFilter(FilterConstants.maxLatencyMs));
        filter.add(new ResultFilters.AmbiguityFilter(FilterConstants.maxAmbiguity));
        filter.add(new ResultFilters.AreaFilter(FilterConstants.minArea, FilterConstants.maxArea));
        filter.add(new ResultFilters.DistanceFilter(FilterConstants.maxSingleTagDistanceMeters));
        if (FilterConstants.maxOdometryDeviationMeters < Double.MAX_VALUE) {
            filter.add(new ResultFilters.OdometryOutlierFilter(
                swerve::getPose,
                FilterConstants.maxOdometryDeviationMeters
            ));
        }
        return new PipelineFilter(filter);
    }

    // ── Interface ─────────────────────────────────────────────────────────────

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}