package com.spartronics4915.frc2026.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.HashMap;
import java.util.Map;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

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

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Common AprilTag localization pipeline. */
public class VisionSubsystem extends SubsystemBase {

    private static VisionSubsystem instance;

    private final SwerveSubsystem swerve;
    private final AprilTagFieldLayout fieldLayout = (Robot.isReal()) ? REAL_APRILTAG_FIELD_LAYOUT : SIM_APRILTAG_FIELD_LAYOUT;
    private final List<CameraIO> cameras = new ArrayList<>();
    private final Map<String, CameraDiagnostics> cameraDiagnostics = new HashMap<>();
    private final VisionSystemSim visionSim;
    private Pose2d latestVisionPose;
    private boolean camerasConfigured;

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
        cameraDiagnostics.put(camera.getName(), new CameraDiagnostics(camera.getName()));
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
        List<CameraObservation> observations = new ArrayList<>();
        for (CameraIO camera : cameras) {
            if (!camera.isEnabled()) {
                CameraDiagnostics diagnostics = cameraDiagnostics.get(camera.getName());
                if (diagnostics != null) {
                    diagnostics.clearObservations();
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
                continue;
            }

            applyVisionMeasurement(observation.cameraName(), observation.estimate());
        }
    }

    private void applyVisionMeasurement(String cameraName, VisionEstimate observation) {
        Matrix<N3, N1> stdDevs = StdDevCalculator.calculate(observation);
        double timestamp = observation.timestamp().in(Seconds);
        String prefix = "Vision/" + cameraName + "/Last";

        swerve.addVisionMeasurement(observation.getPose2d(), timestamp, stdDevs);
        latestVisionPose = observation.getPose2d();
        publishDiagnostics(cameraName, observation);

        SmartDashboard.putNumber(prefix + "TagCount", observation.tagCount());
        SmartDashboard.putNumber(prefix + "TagDistanceMeters", observation.avgTagDistanceMeters());
        SmartDashboard.putNumber(prefix + "Ambiguity", observation.avgTagAmbiguity());
        SmartDashboard.putNumber(prefix + "TagSpanMeters", observation.tagSpanMeters());
        SmartDashboard.putNumber(prefix + "LatencyMs", observation.latencySeconds() * 1000.0);
        SmartDashboard.putNumber(prefix + "StdDevX", stdDevs.get(0, 0));
        SmartDashboard.putNumber(prefix + "StdDevThetaDeg", Math.toDegrees(stdDevs.get(2, 0)));
        SmartDashboard.putNumber(prefix + "Timestamp", timestamp);
    }

    private record CameraObservation(String cameraName, VisionEstimate estimate) {}

    private void publishDiagnostics(String cameraName, VisionEstimate observation) {
        CameraDiagnostics diagnostics = cameraDiagnostics.get(cameraName);
        if (diagnostics == null) {
            return;
        }

        int[] tagIds = observation.tagIds();
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int tagId : tagIds) {
            fieldLayout.getTagPose(tagId).ifPresent(tagPoses::add);
        }

        //diagnostics.seenTagIds.set(tagIds);
        diagnostics.seenTagPoses.set(tagPoses.toArray(Pose3d[]::new));
        diagnostics.estimatedPose.set(observation.pose().toPose2d());
        diagnostics.estimatedPose3d.set(observation.pose());
    }

    private static final class CameraDiagnostics {
        private final IntegerArrayPublisher seenTagIds;
        private final StructArrayPublisher<Pose3d> seenTagPoses;
        private final StructPublisher<Pose2d> estimatedPose;
        private final StructPublisher<Pose3d> estimatedPose3d;

        private CameraDiagnostics(String cameraName) {
            NetworkTable table = NetworkTableInstance.getDefault()
                .getTable("Vision")
                .getSubTable(cameraName);

            seenTagIds = table.getIntegerArrayTopic("SeenTagIds").publish();
            seenTagPoses = table.getStructArrayTopic("SeenTagPoses", Pose3d.struct).publish();
            estimatedPose = table.getStructTopic("EstimatedPose", Pose2d.struct).publish();
            estimatedPose3d = table.getStructTopic("EstimatedPose3d", Pose3d.struct).publish();

            clearObservations();
        }

        private void clearObservations() {
            seenTagIds.set(new long[0]);
            seenTagPoses.set(new Pose3d[0]);
            estimatedPose.set(new Pose2d());
            estimatedPose3d.set(new Pose3d());
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