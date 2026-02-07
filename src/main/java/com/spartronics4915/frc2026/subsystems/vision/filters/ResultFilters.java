package com.spartronics4915.frc2026.subsystems.vision.filters;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

public class ResultFilters {

    /**
     * Filter for a result's latency
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
     * Filter for a result's ambiguity
     */
    public static class AmbiguityFilter implements FilterInterface {
        private final double maxAmbiguity;

        public AmbiguityFilter(double maxAmbiguity) {
            this.maxAmbiguity = maxAmbiguity;
        }

        @Override
        public boolean test(ResultInterface result) {
            return result.getAmbiguity() <= maxAmbiguity;
        }
    }

    /**
     * Filter for a result's average distance to its targets
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
