package com.spartronics4915.frc2026.util.vision;

import static com.spartronics4915.frc2026.Constants.VisionConstants.MAX_OBSERVATION_AGE_SECONDS;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

class StdDevCalculatorTest {
    @Test
    void rejectsStaleObservationsButAcceptsRecentOnes() {
        double now = 10.0;
        VisionEstimate recent = estimate(now - MAX_OBSERVATION_AGE_SECONDS + 0.001);
        VisionEstimate stale = estimate(now - MAX_OBSERVATION_AGE_SECONDS - 0.001);

        assertTrue(StdDevCalculator.isValid(recent, now, 20.0, 10.0));
        assertFalse(StdDevCalculator.isStale(recent, now));
        assertFalse(StdDevCalculator.isValid(stale, now, 20.0, 10.0));
        assertTrue(StdDevCalculator.isStale(stale, now));
    }

    private static VisionEstimate estimate(double timestampSeconds) {
        return new VisionEstimate(
            new int[] {1},
            new Pose3d(2.0, 2.0, 0.0, Rotation3d.kZero),
            Seconds.of(timestampSeconds),
            2.0,
            0.1,
            0.0,
            0.02,
            false);
    }
}
