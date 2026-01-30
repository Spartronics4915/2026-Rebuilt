package com.spartronics4915.frc2026.subsystems.vision.filters;

import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;
import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult.ResultQuality;

public class ResultFilters {

    public static final ResultFilterInterface HAS_TARGETS = new HasTargetsFilter();
    public static final ResultFilterInterface HAS_POSE = new HasPoseFilter();
    
    public static class HasTargetsFilter implements ResultFilterInterface {
        @Override
        public boolean test(CameraResult result) {
            return result.getTargetCount() > 0;
        }
    }

    public static class LatencyFilter implements ResultFilterInterface {
        private final double maxLatencyMs;

        public LatencyFilter(double maxLatencyMs) {
            this.maxLatencyMs = maxLatencyMs;
        }

        @Override
        public boolean test(CameraResult result) {
            return result.getLatencyMs() <= maxLatencyMs;
        }
    }

    public static class AmbiguityFilter implements ResultFilterInterface {
        private final double maxAmbiguity;

        public AmbiguityFilter(double maxAmbiguity) {
            this.maxAmbiguity = maxAmbiguity;
        }

        @Override
        public boolean test(CameraResult result) {
            return result.getAmbiguity() <= maxAmbiguity;
        }
    }

    public static class DistanceFilter implements ResultFilterInterface {
        private final double maxSingleTagDistance;
        private final double maxMultiTagDistance;
        
        public DistanceFilter(double maxSingleTagDistance, double maxMultiTagDistance) {
            this.maxSingleTagDistance = maxSingleTagDistance;
            this.maxMultiTagDistance = maxMultiTagDistance;
        }

        @Override
        public boolean test(CameraResult result) {
            double maxDistance = (result.getTargetCount() == 1) ? maxSingleTagDistance : maxMultiTagDistance;
            return result.getAverageDistanceToTargets() <= maxDistance;
        }
    }

    public static class QualityFilter implements ResultFilterInterface {
        private final ResultQuality minQuality;

        public QualityFilter(ResultQuality minQuality) {
            this.minQuality = minQuality;
        }

        @Override
        public boolean test(CameraResult result) {
            return result.getQuality().ordinal() <= minQuality.ordinal();
        }
    }

    public static class HasPoseFilter implements ResultFilterInterface {
        @Override
        public boolean test(CameraResult result) {
            return result.hasPose();
        }
    }

    public static class MinTargetCountFilter implements ResultFilterInterface {
        private final int minTargets;

        public MinTargetCountFilter(int minTargets) {
            this.minTargets = minTargets;
        }

        @Override
        public boolean test(CameraResult result) {
            return result.getTargetCount() >= minTargets;
        }
    }
}
