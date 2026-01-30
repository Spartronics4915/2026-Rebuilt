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

    public final Matrix<N3, N1> localStdDevs;
    
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
            1,
            VecBuilder.fill(0.5, 0.5, 0.5), 
            VecBuilder.fill(0.5, 0.5, 0.5),
            60,
            18.0, 
            true,  
            3.0,
            2.0,
            true,
            0.65,
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
        Matrix<N3, N1> localStdDevs,
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
        this.localStdDevs = localStdDevs;
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
        private double maxLatencyMs = 100.0;
        private double maxSingleTagDistanceMeters = 4.0;
        private double maxMultiTagDistanceMeters = 8.0;
        private double maxAmbiguityScore = 0.2;
        private Matrix<N3, N1> localStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
        private Matrix<N3, N1> baseGlobalStdDevs = VecBuilder.fill(0.7, 0.7, 0.9);
        private double cameraProcessingFrequencyHz = 100.0;
        private double maxPeriodicTimeMs = 18.0;
        private boolean enableMotionPunishment = true;
        private double velocityPunishmentThreshold = 2.0;
        private double angularVelocityPunishmentThreshold = 2.0;
        private boolean enablePoseFusion = true;
        private double fusionTimestampThreshold = 0.1;
        private int minCamerasForFusion = 2;
        private boolean enableHistoricalValidation = true;
        private double maxPoseJumpMeters = 1.5;

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
                localStdDevs,
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
