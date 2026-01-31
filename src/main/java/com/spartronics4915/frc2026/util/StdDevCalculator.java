package com.spartronics4915.frc2026.util;

import java.util.HashMap;
import java.util.Map;

import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StdDevCalculator {
    
    private final Map<CacheKey, Matrix<N3, N1>> cache;
    private static final int MAX_CACHE_SIZE = 100;

    private static final double DISTANCE_SCALE_FACTOR = 0.02;
    private static final double AMBIGUITY_SCALE_FACTOR = 0.002;
    private static final double MULTI_TAG_REDUCTION = 0.5;   

    public StdDevCalculator() {
        this.cache = new HashMap<>();
    }

    public Matrix<N3, N1> calculate(
        int targetCount,
        double avgDistance,
        double avgAmbiguity,
        VisionConfiguration config
    ) {
        CacheKey key = new CacheKey(targetCount, avgDistance, avgAmbiguity);
        if (cache.containsKey(key)) return cache.get(key);

        Matrix<N3, N1> baseStdDevs = config.baseGlobalStdDevs;
        
        double distanceScale = 1.0 + (avgDistance * DISTANCE_SCALE_FACTOR);
        double ambiguityScale = 1.0 + (avgAmbiguity * AMBIGUITY_SCALE_FACTOR);
        double multiTagScale = (targetCount >= 2) ? MULTI_TAG_REDUCTION : 1.0;
        
        if (targetCount >= 3)  multiTagScale *= 0.8;
        if (targetCount >= 4) multiTagScale *= 0.7;

        double totalScale = distanceScale * ambiguityScale * multiTagScale;

        Matrix<N3, N1> result = VecBuilder.fill(
            baseStdDevs.get(0, 0) * totalScale,
            baseStdDevs.get(1, 0) * totalScale,
            baseStdDevs.get(2, 0) * totalScale * 1.2
        );

        if (cache.size() < MAX_CACHE_SIZE) cache.put(key, result);

        return result;
    }

    public Matrix<N3, N1> applyMotionPunishment(
        Matrix<N3, N1> baseStdDevs,
        double linearVelocity,
        double angularVelocity,
        VisionConfiguration config
    ) {
        
        if (!config.enableMotionPunishment) return baseStdDevs;

        double linearScale = 1.0;
        if (linearVelocity > config.velocityPunishmentThreshold) {
            linearScale = 1.0 + (linearVelocity - config.velocityPunishmentThreshold);
        } else if (linearVelocity < 0.1) linearScale = 0.7;

        double angularScale = 1.0;
        if (angularVelocity > config.angularVelocityThreshold) {
            angularScale = 1.0 + (angularVelocity - config.angularVelocityThreshold) * 2.0;
        }

        double combinedScale = Math.max(linearScale, angularScale);

        return VecBuilder.fill(
            baseStdDevs.get(0, 0) * combinedScale,
            baseStdDevs.get(1, 0) * combinedScale,
            baseStdDevs.get(2, 0) * combinedScale
        );
    }

    public void clearCache() {
        cache.clear();
    }

    private static class CacheKey {
        private final int targetCount;
        private final int distanceBucket; 
        private final int ambiguityBucket;

        CacheKey(int targetCount, double distance, double ambiguity) {
            this.targetCount = targetCount;
            this.distanceBucket = (int) (distance / 0.5);
            this.ambiguityBucket = (int) (ambiguity / 0.05);
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof CacheKey)) return false;
            CacheKey other = (CacheKey) obj;
            return this.targetCount == other.targetCount &&
                this.distanceBucket == other.distanceBucket &&
                this.ambiguityBucket == other.ambiguityBucket;
        }

        @Override
        public int hashCode() {
            return targetCount * 1000 + distanceBucket * 100 + ambiguityBucket;
        }
    }
}
