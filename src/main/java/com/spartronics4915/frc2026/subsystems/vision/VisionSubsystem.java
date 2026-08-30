package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleSupplier;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    private final AprilTagFieldLayout fieldLayout = (Robot.isReal()) ? SIM_APRILTAG_FIELD_LAYOUT : SIM_APRILTAG_FIELD_LAYOUT;
    private final List<CameraIO> cameras = new ArrayList<>();
    private final Map<String, CameraSnapshot> cameraDiagnostics = new HashMap<>();
    private final Map<String, Scope> cameraLogs = new HashMap<>();
    private final VisionSystemSim visionSim;
    private Pose2d latestVisionPose;
    private boolean camerasConfigured;
    private long periodicDurationUs;

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
    public void configureDefaultCameras(java.util.function.DoubleSupplier turretYawDegrees) {
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
            turretYawDegrees);

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
            DoubleSupplier turretYawDegrees
    ) {
        CameraConfig config = CameraConfig.turreted(
            name,
            robotToTurret,
            turretToCamera,
            ignoredTimestamp -> TURRET_CAMERA_YAW_SIGN * turretYawDegrees.getAsDouble()
                + TURNTABLE_ZERO_OFFSET_DEGREES);
        return addLimelightCamera(config);
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
        List<CameraObservation> observations = new ArrayList<>();
        for (CameraIO camera : cameras) {
            CameraSnapshot diagnostics = cameraDiagnostics.get(camera.getName());
            if (diagnostics != null) {
                diagnostics.Enabled = camera.isEnabled();
                diagnostics.SampleTimestampUs = periodicStartUs;
            }
            if (!camera.isEnabled()) {
                if (diagnostics != null) {
                    diagnostics.clearObservation(periodicStartUs, false);
                }
                continue;
            }

            for (VisionEstimate estimate : camera.consumeEstimates()) {
                observations.add(new CameraObservation(camera.getName(), estimate));
            }
        }

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
                    diagnostics.ReceiptTimestampUs = RobotController.getFPGATime();
                }
                continue;
            }

            applyVisionMeasurement(observation.cameraName(), observation.estimate());
        }
        periodicDurationUs = RobotController.getFPGATime() - periodicStartUs;
        outputTelemetry();
    }

    private void applyVisionMeasurement(String cameraName, VisionEstimate observation) {
        Matrix<N3, N1> stdDevs = StdDevCalculator.calculate(observation);
        double timestamp = Utils.fpgaToCurrentTime(observation.timestamp().in(Seconds));
        swerve.addVisionMeasurement(observation.getPose2d(), timestamp, stdDevs);
        latestVisionPose = observation.getPose2d();
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
        LOG.critical.log("PeriodicDurationUs", periodicDurationUs);
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
            cameraLog.critical.log("EstimatedPose", camera.EstimatedPose);
            cameraLog.info.log("TagCount", camera.TagCount);
            cameraLog.info.log("AverageTagDistanceMeters", camera.AverageTagDistanceMeters);
            cameraLog.info.log("AverageTagAmbiguity", camera.AverageTagAmbiguity);
            cameraLog.info.log("TagSpanMeters", camera.TagSpanMeters);
            cameraLog.info.log("LatencyMs", camera.LatencyMs);
            cameraLog.info.log("StdDevXMeters", camera.StdDevXMeters);
            cameraLog.info.log("StdDevThetaDeg", camera.StdDevThetaDeg);
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
}
