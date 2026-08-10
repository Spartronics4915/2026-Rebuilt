package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.List;
import java.util.Optional;

import org.photonvision.simulation.PhotonCameraSim;

import com.spartronics4915.frc2026.subsystems.vision.VisionMeasurement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** A camera that can provide unread AprilTag pose measurements. */
public interface VisionCamera {
    String name();

    void readMeasurements(List<VisionMeasurement> destination);
    Transform3d robotToCamera();

    default boolean isTurreted() { return false; }
    default void updateTurretAngle(Rotation2d turretAngle, double timestampSeconds) {}
    default Optional<PhotonCameraSim> simulationCamera() { return Optional.empty(); }
}
