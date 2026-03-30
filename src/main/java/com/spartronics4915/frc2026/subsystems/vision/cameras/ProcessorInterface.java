package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.List;
import java.util.Optional;

import org.photonvision.simulation.PhotonCameraSim;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Notifier;

/**
 * Common interface for all vision camera processors, regardless of the underlying
 * hardware or library (PhotonVision, Limelight, etc.).
 */
public interface ProcessorInterface {

    // ------ Lifecycle ------

    void start();
    void stop();
    void process();

    // ------ Identity & Geometry ------

    String getCameraName();
    Transform3d getCameraTransform();

    // ------ Result Queue ------

    /**
     * Drains all pending results into {@code destination}.
     * Prefer this over {@link #getResultQueue()}
     */
    void drainResultQueue(List<ResultInterface> destination);

    /** Convenience wrapper; use {@link #drainResultQueue(List)} */
    List<ResultInterface> getResultQueue();

    // ------ Introspection ------

    int getMaxQueueSize();
    Notifier getNotifier();
    double getFrequency();
    boolean isRunning();

    // ------ Configuration ------

    void setPipeline(int newPipelineIndex);
    void setCameraTransform(Transform3d newCameraTransform);

    /**
     * Returns the PhotonVision simulated camera, if this processor supports it.
     */
    default Optional<PhotonCameraSim> getCameraSim() {
        return Optional.empty();
    }

    /**
     * Supplies the current turret yaw and the FPGA timestamp at which it was
     * measured. Called every loop by {@link com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem}
     * when a turret angle supplier is configured.
     *
     * <p>The default implementation is a no-op — fixed cameras ignore this.
     * Override in {@link TurretedPhotonProcessor} (or any future turreted
     * processor) to record the angle for use in per-frame pose correction.
     *
     * @param turretAngle Robot-relative turret yaw (positive = CCW).
     * @param timestamp   FPGA timestamp in seconds.
     */
    default void updateTurretAngle(Rotation2d turretAngle, double timestamp) {}

}