package com.spartronics4915.frc2026.subsystems.vision.hardware;

import com.spartronics4915.frc2026.util.vision.LimelightHelpers;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightSource implements CameraSource {
    private final String name;

    public LimelightSource(String name) {
        this.name = name;
    }

    @Override
    public void updateHeading(Rotation2d yaw) {
        // Feed the gyro to the Limelight for MegaTag 2 calculation
        LimelightHelpers.SetRobotOrientation(
            name, 
            yaw.getDegrees(), 
            0, 
            0, 
            0, 
            0, 
            0
        );
    }

    @Override
    public FiducialObservation[] getObservations() {
        PoseEstimate result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        
        // Return empty array if no tags seen
        if (result == null || result.tagCount == 0) return new FiducialObservation[0];
        
        return new FiducialObservation[] {
            new FiducialObservation(
                result.pose, 
                result.timestampSeconds, 
                result.tagCount, 
                result.avgTagDist, 
                0.0 // MT2 handles ambiguity internally
            )
        };
    }

    @Override public String getName() { 
        return name; 
    }

    @Override public boolean isConnected() { 
        return true; 
    } // Simplified for this example
}