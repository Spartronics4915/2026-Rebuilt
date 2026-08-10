package com.spartronics4915.frc2026.subsystems.vision.processing;

import static com.spartronics4915.frc2026.Constants.VisionConstants.StdDevConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Produces the estimator trust assigned to a single camera frame. */
public final class VisionStdDevs {
    private VisionStdDevs() {}

    public static Matrix<N3, N1> forMeasurement(int tagCount, double averageDistanceMeters) {
        double baseXY = tagCount > 1 ? multiTagXYStdDevMeters : singleTagXYStdDevMeters;
        double distanceScale = Math.max(1.0, averageDistanceMeters);
        return VecBuilder.fill(baseXY * distanceScale, baseXY * distanceScale, thetaStdDevRadians);
    }
}
