package com.spartronics4915.frc2026.util.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Detects wheel slip and collisions by comparing each module's measured drive
 * velocity against its target velocity as commanded by the drivetrain
 */
public class SlipDetector {

    /**
     * Velocity discrepancy (m/s) above which a module is considered slipping.
     * Tune this on your robot:
     *   - Too low  → false positives during aggressive direction changes
     *   - Too high → misses real slip events
     * Start at 0.4 and adjust based on AdvantageScope logs of the "Slip Detected" NT entry.
     */
    private static final double SLIP_THRESHOLD_MPS = 0.4;

    /**
     * Number of consecutive cycles a module must exceed the threshold before slip is flagged.
     * Prevents false positives from single-sample encoder noise.
     */
    private static final int SLIP_DEBOUNCE_CYCLES = 3;

    private final int[] slipCounters = new int[4];
    private boolean slipping = false;

    /**
     * Updates slip detection. Call from the high-frequency odometry Notifier thread.
     *
     * @param moduleStates   Measured states from {@code SwerveDriveState.ModuleStates}
     * @param moduleTargets  Commanded states from {@code SwerveDriveState.ModuleTargets}
     * @return true if any module is currently slipping
     */
    public boolean update(SwerveModuleState[] moduleStates, SwerveModuleState[] moduleTargets) {
        if (moduleStates == null || moduleTargets == null
                || moduleStates.length != 4 || moduleTargets.length != 4) {
            slipping = false;
            return false;
        }

        boolean anySlipping = false;

        for (int i = 0; i < 4; i++) {
            double measured = moduleStates[i].speedMetersPerSecond;
            double expected = moduleTargets[i].speedMetersPerSecond;
            double discrepancy = Math.abs(Math.abs(measured) - Math.abs(expected));

            if (discrepancy > SLIP_THRESHOLD_MPS) {
                slipCounters[i]++;
            } else {
                slipCounters[i] = 0;
            }

            if (slipCounters[i] >= SLIP_DEBOUNCE_CYCLES) {
                anySlipping = true;
            }
        }

        slipping = anySlipping;
        return slipping;
    }

    public boolean isSlipping() {
        return slipping;
    }

    public void reset() {
        for (int i = 0; i < slipCounters.length; i++) slipCounters[i] = 0;
        slipping = false;
    }
}