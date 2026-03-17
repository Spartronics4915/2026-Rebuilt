package com.spartronics4915.frc2026.subsystems.vision;

public class VisionConfiguration {
    
    public final double maxLatencyMs;
    public final double maxSingleTagDistanceMeters;
    public final double maxMultiTagDistanceMeters;
    public final double maxAmbiguityScore;
    public final double maxAnisotropy;
    public final double minArea;
    public final double maxArea;
    public final double maxPeriodicTimeMs;
    public final boolean enableMotionPunishment;
    public final double velocityPunishmentThreshold;
    public final double angularVelocityThreshold;
    public final boolean enablePoseFusion;
    public final double fusionTimestampThreshold;
    public final int minCamerasForFusion;
    public final double fusionOutlierThresholdSigma;
    public final double maxOdometryDeviationMeters;

    public VisionConfiguration() {
        this(
            110,
            6.0, 
            8.0,
            0.9,
            1.9,
            0.05,
            0.70,
            20.0, 
            true,  
            3.0,
            2.0,
            true,
            0.05,
            2,   
            3.0,
            Double.MAX_VALUE
        );
    }

    public VisionConfiguration (
        double maxLatencyMs,
        double maxSingleTagDistanceMeters,
        double maxMultiTagDistanceMeters,
        double maxAmbiguityScore,
        double maxAnisotropy,
        double minArea,
        double maxArea,
        double maxPeriodicTimeMs,
        boolean enableMotionPunishment,
        double velocityPunishmentThreshold,
        double angularVelocityThreshold,
        boolean enablePoseFusion,
        double fusionTimestampThreshold,
        int minCamerasForFusion,
        double fusionOutlierThresholdSigma,
        double maxOdometryDeviationMeters
    ) {
        this.maxLatencyMs = maxLatencyMs;
        this.maxSingleTagDistanceMeters = maxSingleTagDistanceMeters;
        this.maxMultiTagDistanceMeters = maxMultiTagDistanceMeters;
        this.maxAmbiguityScore = maxAmbiguityScore;
        this.maxAnisotropy = maxAnisotropy;
        this.minArea = minArea;
        this.maxArea = maxArea;
        this.maxPeriodicTimeMs = maxPeriodicTimeMs;
        this.enableMotionPunishment = enableMotionPunishment;
        this.velocityPunishmentThreshold = velocityPunishmentThreshold;
        this.angularVelocityThreshold = angularVelocityThreshold;
        this.enablePoseFusion = enablePoseFusion;
        this.fusionTimestampThreshold = fusionTimestampThreshold;
        this.minCamerasForFusion = minCamerasForFusion;
        this.fusionOutlierThresholdSigma = fusionOutlierThresholdSigma;
        this.maxOdometryDeviationMeters = maxOdometryDeviationMeters;
    }

}