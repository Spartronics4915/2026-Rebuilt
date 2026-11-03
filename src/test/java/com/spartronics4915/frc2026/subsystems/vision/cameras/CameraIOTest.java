package com.spartronics4915.frc2026.subsystems.vision.cameras;

import static com.spartronics4915.frc2026.Constants.VisionConstants.MAX_PENDING_ESTIMATES_PER_CAMERA;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.Test;

import com.spartronics4915.frc2026.util.vision.VisionEstimate;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

class CameraIOTest {
    @Test
    void consumingDoesNotWaitForBackendRead() throws Exception {
        CountDownLatch readStarted = new CountDownLatch(1);
        CountDownLatch releaseRead = new CountDownLatch(1);
        CameraIO camera = new TestCamera(() -> {
            readStarted.countDown();
            try {
                assertTrue(releaseRead.await(1, TimeUnit.SECONDS));
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new RuntimeException(exception);
            }
            return List.of(estimate(1));
        });

        ExecutorService executor = Executors.newFixedThreadPool(2);
        try {
            Future<?> update = executor.submit(camera::update);
            assertTrue(readStarted.await(1, TimeUnit.SECONDS));

            Future<List<VisionEstimate>> consume = executor.submit(camera::consumeEstimates);
            assertTrue(consume.get(100, TimeUnit.MILLISECONDS).isEmpty());

            releaseRead.countDown();
            update.get(1, TimeUnit.SECONDS);
            assertEquals(1, camera.consumeEstimates().size());
        } finally {
            releaseRead.countDown();
            executor.shutdownNow();
            camera.stop();
        }
    }

    @Test
    void queueDropsOldestEstimatesAtCapacity() {
        List<VisionEstimate> estimates = new ArrayList<>();
        for (int i = 0; i < MAX_PENDING_ESTIMATES_PER_CAMERA + 3; i++) {
            estimates.add(estimate(i));
        }

        CameraIO camera = new TestCamera(() -> estimates);
        try {
            camera.update();
            assertEquals(MAX_PENDING_ESTIMATES_PER_CAMERA, camera.getPendingEstimateCount());
            assertEquals(MAX_PENDING_ESTIMATES_PER_CAMERA, camera.getMaxPendingEstimateCount());
            assertEquals(3, camera.getDroppedEstimateCount());

            List<VisionEstimate> consumed = camera.consumeEstimates();
            assertEquals(MAX_PENDING_ESTIMATES_PER_CAMERA, consumed.size());
            assertEquals(3.0, consumed.get(0).pose().getX());
            assertEquals(0, camera.getPendingEstimateCount());
        } finally {
            camera.stop();
        }
    }

    private static VisionEstimate estimate(double x) {
        return new VisionEstimate(
            new int[] {1},
            new Pose3d(x, 1.0, 0.0, edu.wpi.first.math.geometry.Rotation3d.kZero),
            Seconds.of(1.0 + x * 0.001),
            1.0,
            0.1,
            0.0,
            0.02,
            false);
    }

    @FunctionalInterface
    private interface EstimateReader {
        List<VisionEstimate> read();
    }

    private static final class TestCamera extends CameraIO {
        private final EstimateReader reader;

        TestCamera(EstimateReader reader) {
            super(new CameraConfig("test", Transform3d.kZero));
            this.reader = reader;
        }

        @Override
        protected List<VisionEstimate> readEstimates() {
            return reader.read();
        }

        @Override
        protected void applyPipeline(CameraPipeline pipeline) {}
    }
}
