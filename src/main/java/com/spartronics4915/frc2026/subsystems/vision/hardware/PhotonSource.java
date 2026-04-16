package com.spartronics4915.frc2026.subsystems.vision.hardware;

import static com.spartronics4915.frc2026.Constants.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSource implements CameraSource {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;

    private record CameraLogger(
        StructPublisher<Pose3d> robotPose,
        StructArrayPublisher<Pose3d> tagPoses,
        IntegerArrayPublisher tagIds,
        DoublePublisher avgDistance,
        DoublePublisher ambiguity,
        DoublePublisher latency,
        DoublePublisher timestamp
    ) {}

    private final CameraLogger loggers;

    public PhotonSource(String name, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        this.loggers = new CameraLogger(
            table.getStructTopic("RobotPose", Pose3d.struct).publish(),
            table.getStructArrayTopic("TagPoses", Pose3d.struct).publish(),
            table.getIntegerArrayTopic("TagIds").publish(),
            table.getDoubleTopic("AvgDistance").publish(),
            table.getDoubleTopic("Ambiguity").publish(),
            table.getDoubleTopic("LatencyMs").publish(),
            table.getDoubleTopic("Timestamp").publish()
        );
    }

    @Override
    public FiducialObservation[] getObservations() {
        return camera.getAllUnreadResults().stream()
            .filter(PhotonPipelineResult::hasTargets)
            .map(result -> {
                Pose3d robotPose;
                int[] ids;
                double ambiguity;
                double distance;

                if (result.multitagResult.isPresent()) {
                    MultiTargetPNPResult multitagResult = result.multitagResult.get();
                    robotPose = new Pose3d()
                        .plus(multitagResult.estimatedPose.best)
                        .plus(robotToCamera.inverse());
                    ids = multitagResult.fiducialIDsUsed.stream()
                        .mapToInt(i -> i)
                        .toArray();
                    ambiguity = multitagResult.estimatedPose.ambiguity;
                    distance = result.targets.stream()
                        .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                        .average().orElse(0.0);
                } else {
                    PhotonTrackedTarget target = result.getBestTarget();
                    Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
                    if (tagPose.isEmpty()) return Optional.empty();

                    robotPose = new Pose3d()
                        .plus(tagPose.get().minus(new Pose3d()))
                        .plus(target.getBestCameraToTarget().inverse())
                        .plus(robotToCamera.inverse());
                    ids = new int[] {target.getFiducialId()};
                    ambiguity = target.getPoseAmbiguity();
                    distance = target.getBestCameraToTarget().getTranslation().getNorm();
                }

                loggers.robotPose.set(robotPose);
                loggers.tagIds.set(Arrays.stream(ids).mapToLong(i -> (long) i).toArray());
                loggers.avgDistance.set(distance);
                loggers.ambiguity.set(ambiguity);
                loggers.timestamp.set(result.getTimestampSeconds());
                loggers.latency.set((Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0);
                loggers.tagPoses.set(Arrays.stream(ids)
                    .mapToObj(id -> aprilTagLayout.getTagPose(id).orElse(new Pose3d()))
                    .toArray(Pose3d[]::new));

                return Optional.of(new FiducialObservation(
                    robotPose.toPose2d(), 
                    result.getTimestampSeconds(), 
                    ids, 
                    distance, 
                    ambiguity
                ));
            }).flatMap(Optional::stream).toArray(FiducialObservation[]::new);
    }

    @Override public void updateHeading(Rotation2d yaw) {}

    @Override public String getName() { 
        return camera.getName(); 
    }

    @Override public boolean isConnected() {
        return camera.isConnected(); 
    }

}