package com.spartronics4915.frc2026.util.swerve;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.minSpeedDetectMPS;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.slipDebounceCycles;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.slipThresholdRPS;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Detects wheel slip and collisions by comparing each module's measured drive
 * velocity against its target velocity as commanded by the drivetrain.
 */
public class SlipDetector {

    private final int[] slipCounters = new int[4];
    private boolean slipping = false;

    public boolean update(SwerveModuleState[] moduleStates, SwerveModuleState[] moduleTargets) {
        if (moduleStates == null || moduleTargets == null
                || moduleStates.length != 4 || moduleTargets.length != 4) {
            slipping = false;
            return false;
        }

        boolean anySlipping = false;

        for (int i = 0; i < 4; i++) {
            double expected = moduleTargets[i].speedMetersPerSecond;

            if (Math.abs(expected) < minSpeedDetectMPS) {
                slipCounters[i] = 0;
                continue;
            }

            double measured = moduleStates[i].speedMetersPerSecond;
            double discrepancy = Math.abs(Math.abs(measured) - Math.abs(expected));

            if (discrepancy > slipThresholdRPS) {
                slipCounters[i]++;
            } else {
                slipCounters[i] = 0;
            }

            if (slipCounters[i] >= slipDebounceCycles) {
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