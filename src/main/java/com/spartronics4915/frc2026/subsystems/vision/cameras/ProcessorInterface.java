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
 *
 * <p>Fixed cameras implement the core methods and ignore {@link #updateTurretAngle}.
 * Turreted cameras (any processor constructed with turret geometry) override
 * {@link #updateTurretAngle} to keep their dynamic transform up to date.
 */
public interface ProcessorInterface {

    // ------ Lifecycle ------

    void start();
    void stop();
    void process();

    // ------ Identity & Geometry ------

    String getCameraName();

    /**
     * Returns the current robot-to-camera transform.
     * For fixed cameras this is constant; for turreted cameras it reflects the
     * latest known turret yaw.
     */
    Transform3d getCameraTransform();

    /** Returns {@code true} if this processor was constructed with turret geometry. */
    default boolean isTurreted() { return false; }

    // ------ Result Queue ------

    void drainResultQueue(List<ResultInterface> destination);
    List<ResultInterface> getResultQueue();

    // ------ Introspection ------

    int getMaxQueueSize();
    Notifier getNotifier();
    double getFrequency();
    boolean isRunning();

    // ------ Configuration ------

    void setPipeline(int newPipelineIndex);
    void setCameraTransform(Transform3d newCameraTransform);

    /** Returns the PhotonVision simulated camera, if supported. */
    default Optional<PhotonCameraSim> getCameraSim() {
        return Optional.empty();
    }

    /**
     * Records the current turret yaw and the FPGA timestamp at which it was measured.
     *
     * <p>Called every loop by {@link com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem}
     * whenever a turret angle supplier is configured. Fixed cameras ignore this (default no-op).
     * Turreted cameras store the sample in a time buffer for interpolation during processing.
     *
     * @param turretAngle Robot-relative turret yaw (CCW positive).
     * @param timestamp   FPGA timestamp in seconds.
     */
    default void updateHeading(Rotation2d turretAngle, double timestamp) {}
}
