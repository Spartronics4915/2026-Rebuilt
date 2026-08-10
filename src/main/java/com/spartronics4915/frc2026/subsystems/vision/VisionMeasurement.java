package com.spartronics4915.frc2026.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** A timestamped robot-pose observation produced by one AprilTag camera frame. */
public record VisionMeasurement(
    String cameraName,
    double timestampSeconds,
    double latencyMs,
    Pose2d robotPose,
    Matrix<N3, N1> standardDeviations,
    List<Integer> tagIds,
    int tagCount,
    double averageAmbiguity,
    double averageArea,
    double averageDistanceMeters
) {
    public VisionMeasurement {
        tagIds = List.copyOf(tagIds);
    }
}
