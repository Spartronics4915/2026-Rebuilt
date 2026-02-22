package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StdDevCalculator {
    
    public StdDevCalculator() {}

    public Matrix<N3, N1> calculate(
        double avgDistance,
        double avgAmbiguity,
        double avgArea,
        double xAnisotropy,
        double yAnisotropy,
        ChassisSpeeds chassisSpeeds,
        double latencyMs,
        int numTags
    ) {
        double latencySeconds = latencyMs / 1000.0;
        
        double distanceFactor = calculateDistanceFactor(avgDistance);
        double ambiguityFactor = calculateAmbiguityFactor(avgAmbiguity);
        double areaFactor = calculateAreaFactor(avgArea);
        double anisotropyFactor = calculateAnisotropyFactor(xAnisotropy, yAnisotropy);
        double latencyFactor = calculateLatencyFactor(latencySeconds);
        double tagCountFactor = calculateTagCountFactor(numTags);

        ChassisSpeeds speeds = chassisSpeeds != null ? chassisSpeeds : new ChassisSpeeds();
        double motionFactor = calculateMotionFactor(speeds);
        
        double xyMultiplier = 
            Math.pow(distanceFactor, distanceWeight) *
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight) *
            Math.pow(anisotropyFactor, anisotropyWeight) *
            Math.pow(motionFactor, motionWeight) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;
        
        double thetaMultiplier = 
            Math.pow(distanceFactor, distanceWeight * 0.7) *
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight * 0.5) *
            Math.pow(anisotropyFactor, anisotropyWeight * 1.3) *
            Math.pow(motionFactor, motionWeight * 1.5) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;
        
        double xyStdDev = baseXYStdDev * xyMultiplier;
        double thetaStdDev = baseThetaStdDev * thetaMultiplier;
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
    
    private double calculateDistanceFactor(double distance) {
        double normalized = distance / 2.0;
        double factor = normalized * normalized;
        
        return Math.min(factor, 10.0);
    }

    private double calculateAmbiguityFactor(double ambiguity) {
        ambiguity = Math.max(0.0, Math.min(ambiguity, 0.25));

        double normalized = ambiguity / 0.1;
        double factor = 1.0 + (normalized * normalized);
        
        return Math.min(factor, 8.0);
    }
    
    private double calculateAreaFactor(double area) {
        area = Math.max(0.001, Math.min(area, 1.0));
        
        double normalized = area / 0.01;
        double factor = 1.0 / Math.sqrt(normalized);
        
        return Math.min(factor, 5.0);
    }
    
    private double calculateAnisotropyFactor(double xAnisotropy, double yAnisotropy) {
        xAnisotropy = Math.max(1.0, Math.min(xAnisotropy, 10.0));
        yAnisotropy = Math.max(1.0, Math.min(yAnisotropy, 10.0));
        
        double combined = Math.sqrt(xAnisotropy * yAnisotropy);
        
        return Math.min(combined, 4.0);
    }
    
    private double calculateMotionFactor(ChassisSpeeds speeds) {
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;
        double omega = speeds.omegaRadiansPerSecond;
        
        double translationalSpeed = Math.sqrt(vx * vx + vy * vy);
        double rotationalSpeed = Math.abs(omega);
        double totalMotion = translationalSpeed + rotationalSpeed;
        double factor = 1.0 + (totalMotion * 0.3);
        
        return Math.min(factor, 3.0);
    }
    
    private double calculateLatencyFactor(double latencySeconds) {
        double factor = 1.0 + (latencySeconds * 5.0);
        return Math.min(factor, 2.0);
    }
    
    private double calculateTagCountFactor(int numTags) {
        return 1.0 / Math.sqrt(Math.max(numTags, 1));
    }
}