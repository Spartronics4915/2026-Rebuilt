package com.spartronics4915.frc2026.subsystems.vision;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.VisionSystemSim;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.cameras.LimelightProcessor;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.processing.PoseFusionEngine;
import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;
import com.spartronics4915.frc2026.subsystems.vision.results.TrackedTag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Vision subsystem that processes AprilTag detections and fuses them with
 * wheel odometry for accurate robot pose estimation.
 */
public class VisionSubsystem extends SubsystemBase {

    // Tracks the most recently SUBMITTED timestamp per camera.
    // Used only for hasValidPose() — NOT for filtering incoming results.
    //
    // Previous versions used this map (or a single global value) as a watermark
    // to skip "already-seen" frames. That caused permanent camera lockout whenever
    // PhotonVision latency fluctuated even slightly: a frame at T with 20 ms
    // latency produces timestamp T-0.020; the next frame with 25 ms latency
    // produces T-0.025 < T-0.020, failing the ">" check, and every subsequent
    // frame from that camera was silently dropped forever.
    //
    // The queue is already DRAINED (each frame consumed exactly once via
    // ConcurrentLinkedQueue.poll()), so no watermark-based deduplication is
    // needed. Removing the filter entirely.
    private final Map<String, Double> lastSubmittedTimestampPerCamera = new HashMap<>();

    private final Map<String, ProcessorInterface> cameras;
    private final AprilTagFieldLayout fieldLayout;
    private final PoseFusionEngine fusionEngine;
    private final SwerveSubsystem swerve;
    private final VisionPoseConsumer poseConsumer;

    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;

    private boolean useVision = true;

    private final List<ResultInterface> rawResultScratch = new ArrayList<>(16);
    private final List<ApriltagResult>  acceptedPerCamera = new ArrayList<>(4);
    private final Pose3d[] tagPoseScratch = new Pose3d[33];

    private static final NetworkTable NT = NetworkTableInstance.getDefault().getTable("vision");

    private final StructPublisher<Pose2d>      fusedPosePublisher      = NT.getStructTopic("FusedPose", Pose2d.struct).publish();
    private final DoublePublisher              xyStdDevPublisher        = NT.getDoubleTopic("XY_StdDev").publish();
    private final DoublePublisher              thetaStdDevPublisher     = NT.getDoubleTopic("Theta_StdDev").publish();
    private final DoublePublisher              cameraCountPublisher     = NT.getDoubleTopic("AcceptedCameras").publish();
    private final BooleanPublisher             usingVisionPublisher     = NT.getBooleanTopic("UsingVision").publish();
    private final StructArrayPublisher<Pose3d> tagPosesPublisher        = NT.getStructArrayTopic("TrackedTagPoses", Pose3d.struct).publish();
    private final StringPublisher              rejectionPublisher       = NT.getStringTopic("LastRejectionReason").publish();
    private final DoublePublisher              rejectionCountPublisher  = NT.getDoubleTopic("RejectionCount").publish();
    private long totalRejections = 0;

    public VisionSubsystem(
        Map<String, ProcessorInterface> cameras,
        AprilTagFieldLayout fieldLayout,
        VisionPoseConsumer poseConsumer,
        SwerveSubsystem swerve
    ) {
        this.cameras = cameras;
        this.fieldLayout = fieldLayout;
        this.poseConsumer = poseConsumer;
        this.swerve = swerve;
        this.fusionEngine = new PoseFusionEngine(swerve::getHistoricalPose);

        this.visionSystemSim = new VisionSystemSim("main");
        this.isSimulation = Robot.isSimulation();

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : cameras.values()) {
            camera.start();
            if (isSimulation) {
                camera.getCameraSim().ifPresent(
                    sim -> visionSystemSim.addCamera(sim, camera.getCameraTransform())
                );
            }
        }
    }

    @Override
    public void periodic() {
        if (isSimulation) {
            visionSystemSim.update(swerve.getPose());
        }

        usingVisionPublisher.set(useVision);
        if (!useVision) return;

        // Seed Limelight with the raw field-frame heading (getPose()), not the
        // alliance-flipped one. getBotPoseEstimate_wpiBlue_MegaTag2 returns poses
        // in the WPILib blue-origin frame and requires the heading in that same
        // frame. On Red alliance getRelativePose() is ~180° off.
        double headingDeg  = swerve.getPose().getRotation().getDegrees();
        double yawRateDegS = Math.toDegrees(swerve.getRobotVelocity().omegaRadiansPerSecond);
        for (ProcessorInterface camera : cameras.values()) {
            if (camera instanceof LimelightProcessor ll) {
                ll.updateHeading(headingDeg, yawRateDegS);
            }
        }

        collectBestPerCamera();

        if (acceptedPerCamera.isEmpty()) return;

        Optional<ApriltagResult> fused = fusionEngine.fuse(acceptedPerCamera);
        fused.ifPresent(this::submit);
    }

    private void collectBestPerCamera() {
        acceptedPerCamera.clear();

        for (ProcessorInterface camera : cameras.values()) {
            rawResultScratch.clear();
            camera.drainResultQueue(rawResultScratch);

            ApriltagResult bestThisCamera = null;
            double bestXYStd = Double.MAX_VALUE;

            for (ResultInterface raw : rawResultScratch) {
                if (!(raw instanceof ApriltagResult result)) continue;

                // No timestamp watermark — see field-level comment.

                Optional<ApriltagResult> accepted = processResult(result, camera.getCameraTransform());
                if (accepted.isEmpty()) continue;

                ApriltagResult acceptedResult = accepted.get();
                double xyStd = acceptedResult.getStdDevs() != null
                    ? acceptedResult.getStdDevs().get(0, 0) : Double.MAX_VALUE;

                if (xyStd < bestXYStd) {
                    bestXYStd = xyStd;
                    bestThisCamera = acceptedResult;
                }
            }

            if (bestThisCamera != null) {
                acceptedPerCamera.add(bestThisCamera);
            }
        }
    }

    private Optional<ApriltagResult> processResult(
        ApriltagResult result,
        Transform3d cameraTransform
    ) {
        if (!isPlausible(result)) {
            reject(result.getSourceName(), "no tracked tags");
            return Optional.empty();
        }

        if (result.isHeadingTrusted()) {
            return processMultiTag(result);
        } else {
            Optional<ApriltagResult> gyro = processGyroBearing(result, cameraTransform);
            if (gyro.isPresent()) return gyro;
            return processMultiTag(result);
        }
    }

    private Optional<ApriltagResult> processMultiTag(ApriltagResult result) {
        if (!result.isMultiTag()) {
            if (result.getAmbiguity() > ambiguityThreshold) {
                reject(result.getSourceName(), "ambiguity " + result.getAmbiguity());
                return Optional.empty();
            }
            if (result.getAverageArea() < minAreaSingleTag) {
                reject(result.getSourceName(), "area too small " + result.getAverageArea());
                return Optional.empty();
            }

            if (result.getAverageArea() < areaForYawCheck) {
                Optional<Pose2d> historicalPose =
                    swerve.getHistoricalPose(result.getTimestampSeconds());

                if (historicalPose.isPresent()) {
                    double yawDiff = Math.abs(MathUtil.angleModulus(
                        historicalPose.get().getRotation().getRadians()
                        - result.getPose().getRotation().getRadians()));
                    if (Math.toDegrees(yawDiff) > maxYawDiff) {
                        reject(result.getSourceName(),
                            "yaw diff " + Math.toDegrees(yawDiff) + " deg");
                        return Optional.empty();
                    }
                }
            }
        }

        double jump = result.getPose().getTranslation()
            .getDistance(swerve.getPose().getTranslation());
        if (jump > maxOdometryJump) {
            reject(result.getSourceName(), "odometry jump " + jump + " m");
            return Optional.empty();
        }

        return Optional.of(result);
    }

    private Optional<ApriltagResult> processGyroBearing(
        ApriltagResult result,
        Transform3d cameraTransform
    ) {
        double maxYaw = swerve.getMaxAbsYawRateInRange(
            result.getTimestampSeconds() - yawLookBack,
            result.getTimestampSeconds()
        ).orElse(Double.POSITIVE_INFINITY);

        if (maxYaw > maxYawRate) {
            reject(result.getSourceName(), "gyro-bearing: yaw rate " + maxYaw);
            return Optional.empty();
        }
        if (result.getSingleTagCameraToTarget().isEmpty()) {
            reject(result.getSourceName(), "gyro-bearing: no camera-to-target transform");
            return Optional.empty();
        }
        if (result.getTrackedTags().isEmpty()) {
            reject(result.getSourceName(), "gyro-bearing: no tracked tags");
            return Optional.empty();
        }

        int tagId = result.getTrackedTags().get(0).fiducialId;

        Optional<Pose3d> maybeTagPose = fieldLayout.getTagPose(tagId);
        if (maybeTagPose.isEmpty()) {
            reject(result.getSourceName(), "gyro-bearing: unknown tag " + tagId);
            return Optional.empty();
        }

        Optional<Pose2d> historicalPose = swerve.getHistoricalPose(result.getTimestampSeconds());
        if (historicalPose.isEmpty()) {
            reject(result.getSourceName(), "gyro-bearing: no historical pose");
            return Optional.empty();
        }

        Rotation2d gyroHeading = historicalPose.get().getRotation();
        Translation2d tagFieldPosition = maybeTagPose.get().toPose2d().getTranslation();

        // Derive the robot-to-tag vector from the raw camera-to-target transform
        // and the camera's known mounting offset, then rotate to field frame using
        // the gyro heading.
        //
        // The old code called fieldToTag.relativeTo(result.getPose()), which applied
        // the inverse of the ambiguous single-tag vision heading — the heading this
        // path is trying to avoid in the first place.
        Transform3d cameraToTarget3d = result.getSingleTagCameraToTarget().get();

        // (X, Y) of camera-to-target in camera frame = horizontal plane projection.
        Translation2d tagInCameraFrame = new Translation2d(
            cameraToTarget3d.getX(), cameraToTarget3d.getY());

        // Rotate to robot frame via camera yaw, then offset by camera position.
        Rotation2d camYawInRobot = new Rotation2d(cameraTransform.getRotation().getZ());
        Translation2d tagInRobotFrame = tagInCameraFrame
            .rotateBy(camYawInRobot)
            .plus(new Translation2d(cameraTransform.getX(), cameraTransform.getY()));

        // Rotate from robot frame to field frame using the trusted gyro heading.
        Translation2d robotToTagInField = tagInRobotFrame.rotateBy(gyroHeading);

        Pose2d gyroBasedPose = new Pose2d(
            tagFieldPosition.minus(robotToTagInField),
            gyroHeading
        );

        double jump = gyroBasedPose.getTranslation()
            .getDistance(swerve.getPose().getTranslation());
        if (jump > maxOdometryJump) {
            reject(result.getSourceName(), "gyro-bearing: odometry jump " + jump + " m");
            return Optional.empty();
        }

        Matrix<N3, N1> stdDevs = StdDevCalculator.calculate(
            result.getAverageArea(), 1, false
        );

        return Optional.of(new ApriltagResult(
            result.getSourceName() + "[gyro]",
            result.getTimestampSeconds(),
            result.getLatencyMs(),
            gyroBasedPose,
            stdDevs,
            result.getTrackedTags(),
            result.getAmbiguity(),
            result.getAverageArea()
        ));
    }

    private void submit(ApriltagResult result) {
        if (result.getStdDevs() == null) return;

        poseConsumer.accept(
            result.getPose(),
            result.getTimestampSeconds(),
            result.getStdDevs()
        );

        // Track submission time per camera for hasValidPose().
        // Strip "[gyro]" suffix so the key always matches the raw camera name.
        for (ApriltagResult accepted : acceptedPerCamera) {
            String cameraName = accepted.getSourceName().replace("[gyro]", "");
            lastSubmittedTimestampPerCamera.merge(
                cameraName, accepted.getTimestampSeconds(), Math::max
            );
        }

        fusedPosePublisher.set(result.getPose());
        xyStdDevPublisher.set(result.getStdDevs().get(0, 0));
        thetaStdDevPublisher.set(result.getStdDevs().get(2, 0));
        cameraCountPublisher.set(acceptedPerCamera.size());
        tagPosesPublisher.set(buildTagPoseArray(result.getTrackedTags()));
    }

    private void reject(String source, String reason) {
        totalRejections++;
        rejectionPublisher.set("[" + source + "] " + reason);
        rejectionCountPublisher.set(totalRejections);
    }

    private static boolean isPlausible(ApriltagResult result) {
        return !result.getTrackedTags().isEmpty();
    }

    private Pose3d[] buildTagPoseArray(List<TrackedTag> tags) {
        int count = 0;
        for (TrackedTag tag : tags) {
            Optional<Pose3d> pose = fieldLayout.getTagPose(tag.fiducialId);
            if (pose.isPresent()) tagPoseScratch[count++] = pose.get();
        }

        Pose3d[] result = new Pose3d[count];
        System.arraycopy(tagPoseScratch, 0, result, 0, count);
        return result;
    }

    public void setUseVision(boolean use) {
        useVision = use;
    }

    public boolean isUsingVision() {
        return useVision;
    }

    public boolean hasValidPose() {
        if (!useVision) return false;
        if (lastSubmittedTimestampPerCamera.isEmpty()) return false;
        double mostRecent = lastSubmittedTimestampPerCamera.values().stream()
            .mapToDouble(Double::doubleValue)
            .max()
            .orElse(0.0);
        return Timer.getFPGATimestamp() - mostRecent <= validPoseStaleness;
    }

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
    }

}