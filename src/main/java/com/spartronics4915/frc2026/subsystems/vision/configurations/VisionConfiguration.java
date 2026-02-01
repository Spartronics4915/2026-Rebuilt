package com.spartronics4915.frc2026.subsystems.vision.configurations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;


public class VisionConfiguration {
    
    public final double maxLatencyMs;
    
    public final double maxSingleTagDistanceMeters;

    public final double maxMultiTagDistanceMeters;
    
    public final double maxAmbiguityScore;
    
    public final Matrix<N3, N1> baseGlobalStdDevs;

    public final double cameraProcessingFrequencyHz;

    public final double maxPeriodicTimeMs;

    public final boolean enableMotionPunishment;
    
    public final double velocityPunishmentThreshold;
    
    public final double angularVelocityThreshold;

    public final boolean enablePoseFusion;
    
    public final double fusionTimestampThreshold;
    
    public final int minCamerasForFusion;

    public final boolean enableHistoricalValidation;

    public final double maxPoseJumpMeters;

    public VisionConfiguration() {
        this(
            150.0,
            7.0, 
            15.0,
            1000,
            VecBuilder.fill(1, 1, 1),
            60,
            20.0, 
            true,  
            4.0,
            3.0,
            true,
            1,
            1,      
            true,
            5.0
        );
    }

    public VisionConfiguration (
        double maxLatencyMs,
        double maxSingleTagDistanceMeters,
        double maxMultiTagDistanceMeters,
        double maxAmbiguityScore,
        Matrix<N3, N1> baseGlobalStdDevs,
        double cameraProcessingFrequencyHz,
        double maxPeriodicTimeMs,
        boolean enableMotionPunishment,
        double velocityPunishmentThreshold,
        double angularVelocityThreshold,
        boolean enablePoseFusion,
        double fusionTimestampThreshold,
        int minCamerasForFusion,
        boolean enableHistoricalValidation,
        double maxPoseJumpMeters
    ) {
        this.maxLatencyMs = maxLatencyMs;
        this.maxSingleTagDistanceMeters = maxSingleTagDistanceMeters;
        this.maxMultiTagDistanceMeters = maxMultiTagDistanceMeters;
        this.maxAmbiguityScore = maxAmbiguityScore;
        this.baseGlobalStdDevs = baseGlobalStdDevs;
        this.cameraProcessingFrequencyHz = cameraProcessingFrequencyHz;
        this.maxPeriodicTimeMs = maxPeriodicTimeMs;
        this.enableMotionPunishment = enableMotionPunishment;
        this.velocityPunishmentThreshold = velocityPunishmentThreshold;
        this.angularVelocityThreshold = angularVelocityThreshold;
        this.enablePoseFusion = enablePoseFusion;
        this.fusionTimestampThreshold = fusionTimestampThreshold;
        this.minCamerasForFusion = minCamerasForFusion;
        this.enableHistoricalValidation = enableHistoricalValidation;
        this.maxPoseJumpMeters = maxPoseJumpMeters;
    }

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private double maxLatencyMs;
        private double maxSingleTagDistanceMeters;
        private double maxMultiTagDistanceMeters;
        private double maxAmbiguityScore;
        private Matrix<N3, N1> baseGlobalStdDevs;
        private double cameraProcessingFrequencyHz;
        private double maxPeriodicTimeMs;
        private boolean enableMotionPunishment;
        private double velocityPunishmentThreshold;
        private double angularVelocityPunishmentThreshold;
        private boolean enablePoseFusion;
        private double fusionTimestampThreshold;
        private int minCamerasForFusion;
        private boolean enableHistoricalValidation;
        private double maxPoseJumpMeters;

        public Builder maxLatencyMs(double maxLatencyMs) {
            this.maxLatencyMs = maxLatencyMs;
            return this;
        }

        public Builder maxSingleTagDistance(double meters) {
            this.maxSingleTagDistanceMeters = meters;
            return this;
        }

        public Builder maxMultiTagDistance(double meters) {
            this.maxMultiTagDistanceMeters = meters;
            return this;
        }

        public Builder enablePoseFusion(boolean enable) {
            this.enablePoseFusion = enable;
            return this;
        }

        public Builder enableMotionPunishment(boolean enable) {
            this.enableMotionPunishment = enable;
            return this;
        }

        public VisionConfiguration build() {
            return new VisionConfiguration(
                maxLatencyMs,
                maxSingleTagDistanceMeters,
                maxMultiTagDistanceMeters,
                maxAmbiguityScore,
                baseGlobalStdDevs,
                cameraProcessingFrequencyHz,
                maxPeriodicTimeMs,
                enableMotionPunishment,
                velocityPunishmentThreshold,
                angularVelocityPunishmentThreshold,
                enablePoseFusion,
                fusionTimestampThreshold,
                minCamerasForFusion,
                enableHistoricalValidation,
                maxPoseJumpMeters
            );
        }
    }
}
