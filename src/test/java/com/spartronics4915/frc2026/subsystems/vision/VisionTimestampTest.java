package com.spartronics4915.frc2026.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class VisionTimestampTest {
    @Test
    void turretHistoryInterpolatesAcrossAngleWrapWithoutAllocatingSamples() {
        VisionSubsystem.TurretAngleHistory history =
            new VisionSubsystem.TurretAngleHistory(4);
        history.add(1.0, Math.toRadians(170.0));
        history.add(2.0, Math.toRadians(-170.0));

        assertEquals(170.0, history.sampleDegrees(0.5), 1e-9);
        assertEquals(180.0, Math.abs(history.sampleDegrees(1.5)), 1e-9);
        assertEquals(-170.0, history.sampleDegrees(2.5), 1e-9);
    }

    @Test
    void turretHistoryRetainsNewestSamplesAtFixedCapacity() {
        VisionSubsystem.TurretAngleHistory history =
            new VisionSubsystem.TurretAngleHistory(3);
        history.add(1.0, Math.toRadians(10.0));
        history.add(2.0, Math.toRadians(20.0));
        history.add(3.0, Math.toRadians(30.0));
        history.add(4.0, Math.toRadians(40.0));

        assertEquals(20.0, history.sampleDegrees(1.0), 1e-9);
        assertEquals(35.0, history.sampleDegrees(3.5), 1e-9);
    }
}
