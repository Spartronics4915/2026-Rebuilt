package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Common interface for all vision pipeline results, regardless of the underlying
 * camera system (PhotonVision, Limelight, etc.).
 */
public interface ResultInterface {

    String getSourceName();
    double getTimestampSeconds();
    double getLatencyMs();
    Pose2d getPose();
    Matrix<N3, N1> getStdDevs();
    List<TrackedTag> getTrackedTags();
    int getTargetCount();
    double getAmbiguity();
    double getAverageArea();
    double getAvgDistance();
    
}