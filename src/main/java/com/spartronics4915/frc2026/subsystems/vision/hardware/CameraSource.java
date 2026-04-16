package com.spartronics4915.frc2026.subsystems.vision.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface CameraSource {
    /** Container for a single vision observation */
    public static record FiducialObservation(
        Pose2d robotPose,
        double timestamp,
        int[] tagIds,
        double averageDistance,
        double ambiguity
    ) {}

    /** Update the camera with the current robot heading (Required for MegaTag 2) */
    void updateHeading(Rotation2d robotYaw);

    /** Fetch the latest observations from the hardware */
    FiducialObservation[] getObservations();
    
    String getName();
    boolean isConnected();
}