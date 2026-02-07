package com.spartronics4915.frc2026.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;


public class VisionConfiguration {
    
    public final double maxLatencyMs;
    public final double maxSingleTagDistanceMeters;
    public final double maxMultiTagDistanceMeters;
    public final double maxAmbiguityScore;
    public final double maxAnisotropy;
    public final double cameraProcessingFrequencyHz;
    public final double maxPeriodicTimeMs;
    public final boolean enableMotionPunishment;
    public final double velocityPunishmentThreshold;
    public final double angularVelocityThreshold;
    public final boolean enablePoseFusion;
    public final double fusionTimestampThreshold;
    public final int minCamerasForFusion;
    public final double fusionOutlierThresholdSigma;
    public final VisionState startingState;

    public VisionConfiguration() {
        this(
            70,
            5.0, 
            10.0,
            0.8,
            1.2,
            100,
            15.0, 
            true,  
            4.0,
            3.0,
            true,
            0.02,
            2,   
            3.0,
            VisionState.GLOBAL
        );
    }

    public VisionConfiguration (
        double maxLatencyMs,
        double maxSingleTagDistanceMeters,
        double maxMultiTagDistanceMeters,
        double maxAmbiguityScore,
        double maxAnisotropy,
        double cameraProcessingFrequencyHz,
        double maxPeriodicTimeMs,
        boolean enableMotionPunishment,
        double velocityPunishmentThreshold,
        double angularVelocityThreshold,
        boolean enablePoseFusion,
        double fusionTimestampThreshold,
        int minCamerasForFusion,
        double fusionOutlierThresholdSigma,
        VisionState startState
    ) {
        this.maxLatencyMs = maxLatencyMs;
        this.maxSingleTagDistanceMeters = maxSingleTagDistanceMeters;
        this.maxMultiTagDistanceMeters = maxMultiTagDistanceMeters;
        this.maxAmbiguityScore = maxAmbiguityScore;
        this.maxAnisotropy = maxAnisotropy;
        this.cameraProcessingFrequencyHz = cameraProcessingFrequencyHz;
        this.maxPeriodicTimeMs = maxPeriodicTimeMs;
        this.enableMotionPunishment = enableMotionPunishment;
        this.velocityPunishmentThreshold = velocityPunishmentThreshold;
        this.angularVelocityThreshold = angularVelocityThreshold;
        this.enablePoseFusion = enablePoseFusion;
        this.fusionTimestampThreshold = fusionTimestampThreshold;
        this.minCamerasForFusion = minCamerasForFusion;
        this.fusionOutlierThresholdSigma = fusionOutlierThresholdSigma;
        this.startingState = startState;
    }

    public enum VisionState {
        GLOBAL, LOCAL, IDLE
    }

}

