package com.spartronics4915.frc2026.subsystems.vision.filters;

import com.spartronics4915.frc2026.subsystems.vision.results.ApriltagResult;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

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
     * A filter for the distance of an {@link ApriltagResult} 
     * that will filter out results if they exceed a certain max distance
     * depending on if its single or multi tag.
     */
    public static class DistanceFilter implements FilterInterface {
        private final double maxSingleTagDistance;
        private final double maxMultiTagDistance;
        
        public DistanceFilter(double maxSingleTagDistance, double maxMultiTagDistance) {
            this.maxSingleTagDistance = maxSingleTagDistance;
            this.maxMultiTagDistance = maxMultiTagDistance;
        }

        @Override
        public boolean test(ResultInterface result) {
            double maxDistance = (result.getTargetCount() == 1) ? maxSingleTagDistance : maxMultiTagDistance;
            return result.getAverageDistanceToTargets() <= maxDistance;
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
     * A filter for the x and y anisotropy of an {@link ApriltagResult} 
     * that will filter out results if they exceed a certain max anisotropy.
     */
    public static class AnisotropyFilter implements FilterInterface {
        private final double maxAnisotropy;

        public AnisotropyFilter(double newMaxAnisotropy) {
            this.maxAnisotropy = newMaxAnisotropy;
        }

        @Override
        public boolean test(ResultInterface result) {
            return result.getXAnisotropy() <= maxAnisotropy 
                && result.getYAnisotropy() <= maxAnisotropy;
        }
    }

}
