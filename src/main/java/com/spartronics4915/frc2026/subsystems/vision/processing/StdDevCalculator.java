package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StdDevCalculator {

    /**
     * Calculates pose estimation standard deviations based on measurement quality factors.
     * Higher values indicate lower confidence. The result is used to weight this measurement
     * against odometry in the pose estimator.
     *
     * @param avgDistance   Average distance to visible tags in meters
     * @param avgAmbiguity  Average pose ambiguity [0.0, 0.25]; lower is better
     * @param avgArea       Average tag area as fraction of frame [0.0, 1.0]; higher is better
     * @param xAnisotropy   Horizontal view angle uncertainty multiplier [1.0, 10.0]
     * @param yAnisotropy   Vertical view angle uncertainty multiplier [1.0, 10.0]
     * @param chassisSpeeds Current robot velocity; may be null (treated as stationary)
     * @param latencyMs     Pipeline latency in milliseconds
     * @param numTags       Number of visible AprilTags; more tags reduce uncertainty
     * @return 3x1 vector of [xStdDev, yStdDev, thetaStdDev] in meters and radians
     */
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
        
        /* 
         * Theta is less sensitive to distance (0.7x) and tag area (0.5x)
         * but more sensitive to anisotropy (1.3x) and motion blur (1.5x)
         * since rotation errors compound with both viewing angle and motion
         */
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
        double normalized = Math.max(distance, 0.1) / 2.0;
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
        double totalMotion = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) 
                       + Math.abs(speeds.omegaRadiansPerSecond);
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