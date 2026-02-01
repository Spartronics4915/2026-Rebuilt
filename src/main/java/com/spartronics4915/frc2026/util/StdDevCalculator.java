package com.spartronics4915.frc2026.util;

import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StdDevCalculator {

    private static final double DISTANCE_SCALE_FACTOR = 0.02;
    private static final double AMBIGUITY_SCALE_FACTOR = 0.002;

    private static final double MULTI_TAG_BASE_REDUCTION = 0.6;
    private static final double MULTI_TAG_REDUCTION_PER_ADDITIONAL = 0.95;
    private static final double MULTI_TAG_MIN_SCALE = 0.1;

    private static final double LINEAR_VELOCITY_SCALE = 1.0;
    private static final double ANGULAR_VELOCITY_SCALE = 2.0;
    private static final double STATIONARY_BONUS = 0.7;
    private static final double STATIONARY_THRESHOLD = 0.1;

    private static final double ROTATION_STD_DEV_MULTIPLIER = 1.2;
    private static final double MIN_ALLOWED_STD_DEV = 0.25;

    public StdDevCalculator() {}

    public Matrix<N3, N1> calculate(
        int targetCount,
        double avgDistance,
        double avgAmbiguity,
        ChassisSpeeds robotSpeeds,
        VisionConfiguration config
    ) {
        double baseScale = calculateBaseScale(targetCount, avgDistance, avgAmbiguity);
        double motionScale = 1.0;

        if (config.enableMotionPunishment && robotSpeeds != null) {
            motionScale = calculateMotionScale(robotSpeeds, config);
        }

        double totalScale = baseScale * motionScale;
        Matrix<N3, N1> baseStdDevs = config.baseGlobalStdDevs;

        double xStdDev = Math.max(baseStdDevs.get(0, 0) * totalScale, MIN_ALLOWED_STD_DEV);
        double yStdDev = Math.max(baseStdDevs.get(1, 0) * totalScale, MIN_ALLOWED_STD_DEV);
        double thetaStdDev = Math.max(
            baseStdDevs.get(2, 0) * totalScale * ROTATION_STD_DEV_MULTIPLIER, 
            MIN_ALLOWED_STD_DEV
        );

        return VecBuilder.fill(xStdDev, yStdDev, thetaStdDev);
    }

    private double calculateBaseScale(int targetCount, double avgDistance, double avgAmbiguity) {
        double distanceScale = 1.0 + (avgDistance * DISTANCE_SCALE_FACTOR);
        double ambiguityScale = 1.0 + (avgAmbiguity * AMBIGUITY_SCALE_FACTOR);
        double multiTagScale = getMultiTagFactor(targetCount);

        return distanceScale * ambiguityScale * multiTagScale;
    }

    private double getMultiTagFactor(int targetCount) {
        if (targetCount < 2) {
            return 1.0;
        }

        double scale = MULTI_TAG_BASE_REDUCTION;

        if (targetCount > 2) {
            scale *= Math.pow(MULTI_TAG_REDUCTION_PER_ADDITIONAL, targetCount - 2);
        }

        return Math.max(scale, MULTI_TAG_MIN_SCALE);
    }

    private double calculateMotionScale(ChassisSpeeds speeds, VisionConfiguration config) {
        double linearVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularVelocity = Math.abs(speeds.omegaRadiansPerSecond);

        if (linearVelocity < STATIONARY_THRESHOLD) {
            return STATIONARY_BONUS;
        }

        double linearScale = 1.0;
        if (linearVelocity > config.velocityPunishmentThreshold) {
            double excess = linearVelocity - config.velocityPunishmentThreshold;
            linearScale += (excess * LINEAR_VELOCITY_SCALE);
        }

        double angularScale = 1.0;
        if (angularVelocity > config.angularVelocityThreshold) {
            double excess = angularVelocity - config.angularVelocityThreshold;
            angularScale += (excess * ANGULAR_VELOCITY_SCALE);
        }

        return Math.max(linearScale, angularScale);
    }
}