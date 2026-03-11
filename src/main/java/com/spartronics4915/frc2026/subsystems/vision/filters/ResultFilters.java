package com.spartronics4915.frc2026.subsystems.vision.filters;

import java.util.function.Supplier;

import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

import edu.wpi.first.math.geometry.Pose2d;

public class ResultFilters {

    /**
     * A filter for the latency of an {@link ApriltagResult} 
     * that will filter out results if they exceed a certain max latency.
     */
    public static class LatencyFilter implements FilterInterface {
        private final double maxLatencyMs;

        public LatencyFilter(double maxLatencyMs) {
            this.maxLatencyMs = maxLatencyMs;
        }

        @Override
        public boolean test(ResultInterface result) {
            return result.getLatencyMs() <= maxLatencyMs;
        }
    }

    /**
     * A filter for the ambiguity of an {@link ApriltagResult} 
     * that will filter out results if they exceed a certain max ambiguity.
     */
    public static class AmbiguityFilter implements FilterInterface {
        private final double maxAmbiguity;

        public AmbiguityFilter(double maxAmbiguity) {
            this.maxAmbiguity = maxAmbiguity;
        }

        @Override
        public boolean test(ResultInterface result) {
            if (result.getTargetCount() > 1) return true;
            return result.getAmbiguity() <= maxAmbiguity;
        }
    }   

    /**
     * A filter for the area (size in camera frame) of an {@link ApriltagResult} 
     * that will filter out results if they exceed a certain max or min area.
     */
    public static class AreaFilter implements FilterInterface {
        private final double minArea;
        private final double maxArea;

        public AreaFilter(double newMinArea, double newMaxArea) {
            this.minArea = newMinArea;
            this.maxArea = newMaxArea;
        }

        @Override
        public boolean test(ResultInterface result) {
            return result.getAverageArea() >= minArea && result.getAverageArea() <= maxArea;
        }
    }

    /**
     * A filter that rejects any vision result whose estimated pose is farther than
     * {@code maxDeviationMeters} from the current odometry pose
     */
    public static class OdometryOutlierFilter implements FilterInterface {
        private final Supplier<Pose2d> odometrySupplier;
        private final double maxDeviationMeters;

        public OdometryOutlierFilter(Supplier<Pose2d> odometrySupplier, double maxDeviationMeters) {
            this.odometrySupplier = odometrySupplier;
            this.maxDeviationMeters = maxDeviationMeters;
        }

        @Override
        public boolean test(ResultInterface result) {
            Pose2d reference = odometrySupplier.get();
            if (reference == null) return true;
            double dist = result.getPose().getTranslation().getDistance(reference.getTranslation());
            return dist <= maxDeviationMeters;
        }
    }

}