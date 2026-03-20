package com.spartronics4915.frc2026.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Exponential moving average filter for robot pose.
 *
 * <p>Only X and Y are smoothed. Rotation is always taken from the raw input
 * because the Pigeon2 already provides excellent short-term heading stability
 * and filtering heading introduces phase lag that hurts accuracy.
 *
 * <p>Intended for use by {@link com.spartronics4915.frc2026.subsystems.control.AutoAimController}
 * and any other consumer that needs smooth, low-jitter pose at the cost of a
 * small lag (typically 50–200 ms depending on alpha).
 *
 * <p>Alpha tuning guide (at 50 Hz periodic):
 * <ul>
 *   <li>0.05 → ~1.0 s lag, very smooth
 *   <li>0.15 → ~300 ms lag, good for auto-aim
 *   <li>0.30 → ~150 ms lag, responsive but still smoother than raw
 * </ul>
 */
public class MovingAveragePose {

    private final double alpha;

    private double filteredX;
    private double filteredY;
    private boolean initialized;

    /**
     * @param alpha Smoothing factor in (0, 1]. Higher = more responsive, less smooth.
     *              Typical range 0.10–0.25 for auto-aim consumers.
     */
    public MovingAveragePose(double alpha) {
        this.alpha = Math.max(1e-6, Math.min(1.0, alpha));
        this.initialized = false;
    }

    /**
     * Updates the filter with a new raw pose and returns the smoothed result.
     * On the first call the filter is seeded to the raw value with no lag.
     *
     * @param raw Latest raw (fused odometry) pose.
     * @return Smoothed pose. Rotation is taken directly from {@code raw}.
     */
    public Pose2d calculate(Pose2d raw) {
        if (!initialized) {
            filteredX = raw.getX();
            filteredY = raw.getY();
            initialized = true;
        } else {
            filteredX = alpha * raw.getX() + (1.0 - alpha) * filteredX;
            filteredY = alpha * raw.getY() + (1.0 - alpha) * filteredY;
        }
        return new Pose2d(new Translation2d(filteredX, filteredY), raw.getRotation());
    }

    /**
     * Snaps the filter to a new pose instantly, eliminating any accumulated lag.
     * Call this when the robot's pose is hard-reset (e.g. at autonomous init).
     */
    public void reset(Pose2d pose) {
        filteredX = pose.getX();
        filteredY = pose.getY();
        initialized = true;
    }

    /** Clears all history. Next {@link #calculate} call will re-initialize. */
    public void reset() {
        initialized = false;
    }
    
}