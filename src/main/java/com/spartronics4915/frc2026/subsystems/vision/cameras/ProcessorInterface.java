package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.List;
import java.util.Optional;

import org.photonvision.simulation.PhotonCameraSim;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Common interface for all vision camera processors.
 */
public interface ProcessorInterface {

    void start();
    void stop();
    void process();

    String getCameraName();
    Transform3d getCameraTransform();

    void drainResultQueue(List<ResultInterface> destination);
    List<ResultInterface> getResultQueue();

    int getMaxQueueSize();
    Notifier getNotifier();
    double getFrequency();
    boolean isRunning();

    void setPipeline(int newPipelineIndex);
    void setCameraTransform(Transform3d newCameraTransform);

    default boolean isTurreted() { return false; }
    default Optional<PhotonCameraSim> getCameraSim() {
        return Optional.empty();
    }

    void setRobotOrientation(double headingDegrees);
    void updateTurretAngle(Rotation2d turretAngle, double timestamp);
    default void updateHeading(Rotation2d angle, double timestamp) {
        updateTurretAngle(angle, timestamp);
    }
}
