package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.HashMap;
import java.util.Map;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import com.ctre.phoenix6.Utils;
import com.spartronics4915.frc2026.Constants;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.cameras.CameraIO;
import com.spartronics4915.frc2026.subsystems.vision.cameras.CameraIO.CameraConfig;
import com.spartronics4915.frc2026.subsystems.vision.cameras.limelight.LimelightCameraIO;
import com.spartronics4915.frc2026.subsystems.vision.cameras.photon.PhotonCameraIO;
import com.spartronics4915.frc2026.subsystems.vision.cameras.photon.SimulatedCameraIO;
import com.spartronics4915.frc2026.util.vision.VisionEstimate;
import com.spartronics4915.frc2026.util.vision.StdDevCalculator;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Common AprilTag localization pipeline. */
public class VisionSubsystem extends SubsystemBase {
    private static final Scope LOG = Telemetry.scope("Vision");

    private static VisionSubsystem instance;

    private final SwerveSubsystem swerve;
    private final AprilTagFieldLayout fieldLayout = Robot.isReal()
        ? REAL_APRILTAG_FIELD_LAYOUT
        : SIM_APRILTAG_FIELD_LAYOUT;
    private final List<CameraIO> cameras = new ArrayList<>();
    private final Map<String, CameraSnapshot> cameraDiagnostics = new HashMap<>();
    private final Map<String, Scope> cameraLogs = new HashMap<>();
    private final VisionSystemSim visionSim;
    private final TurretAngleHistory turretAngleHistory = new TurretAngleHistory(64);
    private Pose2d latestVisionPose;
    private boolean camerasConfigured;
    private long periodicDurationUs;
    private long collectionDurationUs;
    private long validationFusionDurationUs;
    private long telemetryDurationUs;

    private VisionSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
        visionSim = Robot.isSimulation() ? new VisionSystemSim("vision") : null;

        if (visionSim != null) {
            visionSim.addAprilTags(fieldLayout);
            SmartDashboard.putData("Vision/Sim Field", visionSim.getDebugField());
        }
    }

    public static VisionSubsystem getInstance(SwerveSubsystem swerve) {
        if (instance == null) {
            instance = new VisionSubsystem(swerve);
        } else if (instance.swerve != swerve) {
            throw new IllegalStateException("VisionSubsystem was initialized with a different SwerveSubsystem instance.");
        }
        return instance;
    }


    /** Configures the robot's three fixed PhotonVision cameras and turreted MegaTag1 Limelight. */
    public void configureDefaultCameras() {
        if (camerasConfigured) {
            return;
        }

        addPhotonCamera(new CameraConfig("evan", frontCameraTransform));
        addPhotonCamera(new CameraConfig("val", backCameraTransform));
        addPhotonCamera(new CameraConfig("daniil", rioCameraTransform));

        Transform3d robotToTurret = new Transform3d(
            Constants.SuperstructureConstants.turretTranslation3D,
            new Rotation3d());

        addTurretedLimelight(
            "argos",
            robotToTurret,
            turretToCamera,
            turretAngleHistory::sampleDegrees);

        camerasConfigured = true;
    }

    public Pose2d getVisionPose() {
        return latestVisionPose != null ? latestVisionPose : swerve.getPose();
    }

    public void addCamera(CameraIO camera) {
        cameras.add(camera);
        cameraDiagnostics.put(camera.getName(), new CameraSnapshot());
        cameraLogs.put(camera.getName(), LOG.child(camera.getName()));
        SmartDashboard.putData("Vision/" + camera.getName(), camera);
        camera.start();
    }

    public CameraIO addPhotonCamera(CameraConfig config) {
        CameraIO camera = Robot.isSimulation()
            ? new SimulatedCameraIO(
                config,
                fieldLayout,
                requireSimulation(),
                createSimulationCameraProperties())
            : new PhotonCameraIO(config, fieldLayout);
        addCamera(camera);
        return camera;
    }

    public CameraIO addLimelightCamera(CameraConfig config) {
        CameraIO camera = Robot.isSimulation()
            ? new SimulatedCameraIO(
                config,
                fieldLayout,
                requireSimulation(),
                createSimulationCameraProperties())
            : new LimelightCameraIO(config, fieldLayout);
        addCamera(camera);
        return camera;
    }

    /** Adds a real MegaTag1 Limelight whose robot-space pose follows the current turret angle. */
    public CameraIO addTurretedLimelight(
            String name,
            Transform3d robotToTurret,
            Transform3d turretToCamera,
            DoubleUnaryOperator turretYawDegreesAtTimestamp
    ) {
        CameraConfig config = CameraConfig.turreted(
            name,
            robotToTurret,
            turretToCamera,
            timestamp -> TURRET_CAMERA_YAW_SIGN
                * turretYawDegreesAtTimestamp.applyAsDouble(timestamp)
                + TURNTABLE_ZERO_OFFSET_DEGREES);
        return addLimelightCamera(config);
    }

    public void recordTurretAngle(Rotation2d angle, double timestampSeconds) {
        turretAngleHistory.add(timestampSeconds, angle.getRadians());
    }

    /** Adds a PhotonVision camera whose turret angle can be reconstructed at each frame timestamp. */
    public CameraIO addTimestampedTurretedPhotonCamera(
        String name,
        Transform3d robotToTurret,
        Transform3d turretToCamera,
        DoubleUnaryOperator turretYawDegreesAtTimestamp
    ) {
        CameraConfig config = CameraConfig.turreted(
            name,
            robotToTurret,
            turretToCamera,
            timestamp -> TURRET_CAMERA_YAW_SIGN
                * turretYawDegreesAtTimestamp.applyAsDouble(timestamp)
                + TURNTABLE_ZERO_OFFSET_DEGREES);
        return addPhotonCamera(config);
    }

    public SimulatedCameraIO addSimulatedCamera(
        CameraConfig config,
        SimCameraProperties cameraProperties
    ) {
        SimulatedCameraIO camera = new SimulatedCameraIO(
            config,
            fieldLayout,
            requireSimulation(),
            cameraProperties);
        addCamera(camera);
        return camera;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    @Override
    public void periodic() {
        long periodicStartUs = RobotController.getFPGATime();
        long collectionStartUs = periodicStartUs;
        List<CameraObservation> observations = new ArrayList<>();
        for (CameraIO camera : cameras) {
            CameraSnapshot diagnostics = cameraDiagnostics.get(camera.getName());
            boolean enabled = camera.isEnabled();
            if (diagnostics != null) {
                diagnostics.updateCameraHealth(camera);
                diagnostics.Enabled = enabled;
                diagnostics.SampleTimestampUs = periodicStartUs;
            }
            if (!enabled) {
                if (diagnostics != null) {
                    diagnostics.clearObservation(periodicStartUs, false);
                }
                continue;
            }

            for (VisionEstimate estimate : camera.consumeEstimates()) {
                observations.add(new CameraObservation(camera.getName(), estimate));
            }
        }
        collectionDurationUs = RobotController.getFPGATime() - collectionStartUs;

        long validationFusionStartUs = RobotController.getFPGATime();
        observations.sort(Comparator.comparingDouble(
            observation -> observation.estimate().timestamp().in(Seconds)));

        double now = Timer.getFPGATimestamp();
        for (CameraObservation observation : observations) {
            if (!StdDevCalculator.isValid(
                    observation.estimate(),
                    now,
                    fieldLayout.getFieldLength(),
                    fieldLayout.getFieldWidth())) {
                CameraSnapshot diagnostics = cameraDiagnostics.get(observation.cameraName());
                if (diagnostics != null) {
                    diagnostics.Accepted = false;
                    diagnostics.RejectedObservationCount++;
                    if (StdDevCalculator.isStale(observation.estimate(), now)) {
                        diagnostics.StaleObservationCount++;
                    }
                    diagnostics.ReceiptTimestampUs = RobotController.getFPGATime();
                }
                continue;
            }

            applyVisionMeasurement(observation.cameraName(), observation.estimate());
        }
        validationFusionDurationUs = RobotController.getFPGATime() - validationFusionStartUs;

        long telemetryStartUs = RobotController.getFPGATime();
        outputTelemetry();
        telemetryDurationUs = RobotController.getFPGATime() - telemetryStartUs;
        periodicDurationUs = RobotController.getFPGATime() - periodicStartUs;

        // These final timing writes intentionally sit outside the measured telemetry block so the
        // current cycle, rather than the previous cycle, is published.
        LOG.critical.log("PeriodicDurationUs", periodicDurationUs);
        LOG.critical.log("CollectionDurationUs", collectionDurationUs);
        LOG.critical.log("ValidationFusionDurationUs", validationFusionDurationUs);
        LOG.critical.log("TelemetryDurationUs", telemetryDurationUs);
    }

    private void applyVisionMeasurement(String cameraName, VisionEstimate observation) {
        Matrix<N3, N1> stdDevs = StdDevCalculator.calculate(observation);
        double timestamp = Utils.fpgaToCurrentTime(observation.timestamp().in(Seconds));
        Pose2d pose = observation.getPose2d();
        swerve.addVisionMeasurement(pose, timestamp, stdDevs);
        latestVisionPose = pose;
        updateDiagnostics(cameraName, observation, stdDevs);
    }

    private record CameraObservation(String cameraName, VisionEstimate estimate) {}

    private void updateDiagnostics(
        String cameraName,
        VisionEstimate observation,
        Matrix<N3, N1> stdDevs
    ) {
        CameraSnapshot diagnostics = cameraDiagnostics.get(cameraName);
        if (diagnostics == null) {
            return;
        }

        int[] tagIds = observation.tagIds();
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int tagId : tagIds) {
            fieldLayout.getTagPose(tagId).ifPresent(tagPoses::add);
        }

        diagnostics.SeenTagIds = new long[tagIds.length];
        for (int i = 0; i < tagIds.length; i++) {
            diagnostics.SeenTagIds[i] = tagIds[i];
        }
        diagnostics.SeenTagPoses = tagPoses.toArray(Pose3d[]::new);
        diagnostics.EstimatedPose = observation.pose().toPose2d();
        diagnostics.EstimatedPose3d = observation.pose();
        diagnostics.TagCount = observation.tagCount();
        diagnostics.AverageTagDistanceMeters = observation.avgTagDistanceMeters();
        diagnostics.AverageTagAmbiguity = observation.avgTagAmbiguity();
        diagnostics.TagSpanMeters = observation.tagSpanMeters();
        diagnostics.LatencyMs = observation.latencySeconds() * 1000.0;
        diagnostics.StdDevXMeters = stdDevs.get(0, 0);
        diagnostics.StdDevThetaDeg = Math.toDegrees(stdDevs.get(2, 0));
        diagnostics.CaptureTimestampUs = Math.round(observation.timestamp().in(Seconds) * 1_000_000.0);
        diagnostics.ReceiptTimestampUs = RobotController.getFPGATime();
        diagnostics.SampleTimestampUs = diagnostics.ReceiptTimestampUs;
        diagnostics.Enabled = true;
        diagnostics.Accepted = true;
        diagnostics.AcceptedObservationCount++;
    }

    public long getPeriodicDurationUs() {
        return periodicDurationUs;
    }

    private void outputTelemetry() {
        for (Map.Entry<String, CameraSnapshot> entry : cameraDiagnostics.entrySet()) {
            Scope cameraLog = cameraLogs.get(entry.getKey());
            CameraSnapshot camera = entry.getValue();
            cameraLog.critical.log("SampleTimestampUs", camera.SampleTimestampUs);
            cameraLog.critical.log("CaptureTimestampUs", camera.CaptureTimestampUs);
            cameraLog.critical.log("ReceiptTimestampUs", camera.ReceiptTimestampUs);
            cameraLog.critical.log("Enabled", camera.Enabled);
            cameraLog.critical.log("Accepted", camera.Accepted);
            cameraLog.critical.log("AcceptedObservationCount", camera.AcceptedObservationCount);
            cameraLog.critical.log("RejectedObservationCount", camera.RejectedObservationCount);
            cameraLog.critical.log("StaleObservationCount", camera.StaleObservationCount);
            cameraLog.critical.log("EstimatedPose", camera.EstimatedPose);
            cameraLog.critical.log("PendingEstimateCount", camera.PendingEstimateCount);
            cameraLog.critical.log("DroppedEstimateCount", camera.DroppedEstimateCount);
            cameraLog.info.log("TagCount", camera.TagCount);
            cameraLog.info.log("AverageTagDistanceMeters", camera.AverageTagDistanceMeters);
            cameraLog.info.log("AverageTagAmbiguity", camera.AverageTagAmbiguity);
            cameraLog.info.log("TagSpanMeters", camera.TagSpanMeters);
            cameraLog.info.log("LatencyMs", camera.LatencyMs);
            cameraLog.info.log("StdDevXMeters", camera.StdDevXMeters);
            cameraLog.info.log("StdDevThetaDeg", camera.StdDevThetaDeg);
            cameraLog.info.log("LastReadDurationUs", camera.LastReadDurationUs);
            cameraLog.info.log("MaxReadDurationUs", camera.MaxReadDurationUs);
            cameraLog.info.log("ReadOverrunCount", camera.ReadOverrunCount);
            cameraLog.info.log("LastQueueWaitDurationUs", camera.LastQueueWaitDurationUs);
            cameraLog.info.log("MaxQueueWaitDurationUs", camera.MaxQueueWaitDurationUs);
            cameraLog.debug.log("MaxPendingEstimateCount", camera.MaxPendingEstimateCount);
            cameraLog.debug.log("SeenTagIds", camera.SeenTagIds);
            cameraLog.debug.log("SeenTagPoses", camera.SeenTagPoses);
            cameraLog.debug.log("EstimatedPose3d", camera.EstimatedPose3d);
        }
    }

    private static final class CameraSnapshot {
        private static final long[] NO_TAG_IDS = new long[0];
        private static final Pose3d[] NO_TAG_POSES = new Pose3d[0];
        long SampleTimestampUs;
        long CaptureTimestampUs;
        long ReceiptTimestampUs;
        boolean Enabled;
        boolean Accepted;
        long AcceptedObservationCount;
        long RejectedObservationCount;
        long StaleObservationCount;
        long PendingEstimateCount;
        long MaxPendingEstimateCount;
        long DroppedEstimateCount;
        long LastReadDurationUs;
        long MaxReadDurationUs;
        long ReadOverrunCount;
        long LastQueueWaitDurationUs;
        long MaxQueueWaitDurationUs;
        long[] SeenTagIds = NO_TAG_IDS;
        Pose3d[] SeenTagPoses = NO_TAG_POSES;
        Pose2d EstimatedPose = new Pose2d();
        Pose3d EstimatedPose3d = new Pose3d();
        long TagCount;
        double AverageTagDistanceMeters;
        double AverageTagAmbiguity;
        double TagSpanMeters;
        double LatencyMs;
        double StdDevXMeters;
        double StdDevThetaDeg;

        void updateCameraHealth(CameraIO camera) {
            PendingEstimateCount = camera.getPendingEstimateCount();
            MaxPendingEstimateCount = camera.getMaxPendingEstimateCount();
            DroppedEstimateCount = camera.getDroppedEstimateCount();
            LastReadDurationUs = camera.getLastReadDurationUs();
            MaxReadDurationUs = camera.getMaxReadDurationUs();
            ReadOverrunCount = camera.getReadOverrunCount();
            LastQueueWaitDurationUs = camera.getLastQueueWaitDurationUs();
            MaxQueueWaitDurationUs = camera.getMaxQueueWaitDurationUs();
        }

        void clearObservation(long sampleTimestampUs, boolean enabled) {
            SampleTimestampUs = sampleTimestampUs;
            ReceiptTimestampUs = sampleTimestampUs;
            Enabled = enabled;
            Accepted = false;
            SeenTagIds = NO_TAG_IDS;
            SeenTagPoses = NO_TAG_POSES;
            EstimatedPose = new Pose2d();
            EstimatedPose3d = new Pose3d();
            TagCount = 0;
            AverageTagDistanceMeters = 0;
            AverageTagAmbiguity = 0;
            TagSpanMeters = 0;
            LatencyMs = 0;
            StdDevXMeters = 0;
            StdDevThetaDeg = 0;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim == null) {
            return;
        }

        for (CameraIO camera : cameras) {
            if (camera instanceof SimulatedCameraIO simulatedCamera) {
                simulatedCamera.updateSimulationTransform();
            }
        }

        //visionSim.update(swerve.getSimulatedTruthPose());
        visionSim.update(swerve.getPose());
        visionSim.getDebugField()
            .getObject("TruthRobot")
            .setPose(swerve.getPose());
        visionSim.getDebugField()
            .getObject("EstimatedRobot")
            .setPose(swerve.getPose());
    }

    private VisionSystemSim requireSimulation() {
        if (visionSim == null) {
            throw new IllegalStateException("A simulated camera can only be added in simulation.");
        }
        return visionSim;
    }

    /** Fixed-size, allocation-free history used by camera worker threads. */
    static final class TurretAngleHistory {
        private final double[] timestampsSeconds;
        private final double[] yawRadians;
        private int nextIndex;
        private int size;

        TurretAngleHistory(int capacity) {
            if (capacity < 2) {
                throw new IllegalArgumentException("Turret angle history needs at least two samples.");
            }
            timestampsSeconds = new double[capacity];
            yawRadians = new double[capacity];
        }

        synchronized void add(double timestampSeconds, double angleRadians) {
            if (!Double.isFinite(timestampSeconds) || !Double.isFinite(angleRadians)) {
                return;
            }

            if (size > 0) {
                int newestIndex = Math.floorMod(nextIndex - 1, timestampsSeconds.length);
                double newestTimestamp = timestampsSeconds[newestIndex];
                if (timestampSeconds < newestTimestamp) {
                    return;
                }
                if (timestampSeconds == newestTimestamp) {
                    yawRadians[newestIndex] = angleRadians;
                    return;
                }
            }

            timestampsSeconds[nextIndex] = timestampSeconds;
            yawRadians[nextIndex] = angleRadians;
            nextIndex = (nextIndex + 1) % timestampsSeconds.length;
            size = Math.min(size + 1, timestampsSeconds.length);
        }

        synchronized double sampleDegrees(double timestampSeconds) {
            if (size == 0) {
                return 0.0;
            }

            int oldestIndex = Math.floorMod(nextIndex - size, timestampsSeconds.length);
            int newestIndex = Math.floorMod(nextIndex - 1, timestampsSeconds.length);
            if (timestampSeconds <= timestampsSeconds[oldestIndex]) {
                return Math.toDegrees(yawRadians[oldestIndex]);
            }
            if (timestampSeconds >= timestampsSeconds[newestIndex]) {
                return Math.toDegrees(yawRadians[newestIndex]);
            }

            int previousIndex = oldestIndex;
            for (int offset = 1; offset < size; offset++) {
                int currentIndex = (oldestIndex + offset) % timestampsSeconds.length;
                double currentTimestamp = timestampsSeconds[currentIndex];
                if (currentTimestamp >= timestampSeconds) {
                    double previousTimestamp = timestampsSeconds[previousIndex];
                    double interpolation = (timestampSeconds - previousTimestamp)
                        / (currentTimestamp - previousTimestamp);
                    double deltaRadians = MathUtil.angleModulus(
                        yawRadians[currentIndex] - yawRadians[previousIndex]);
                    return Math.toDegrees(yawRadians[previousIndex] + deltaRadians * interpolation);
                }
                previousIndex = currentIndex;
            }

            return Math.toDegrees(yawRadians[newestIndex]);
        }
    }
}
