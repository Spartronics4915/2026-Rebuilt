package com.spartronics4915.frc2026.util.vision;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;

/** Timestamped AprilTag pose estimate plus quality metrics used by the vision processor. */
public record VisionEstimate(
    int[] tagIds,
    Pose3d pose,
    Time timestamp,
    double avgTagDistanceMeters,
    double avgTagAmbiguity,
    double tagSpanMeters,
    double latencySeconds,
    boolean useVisionRotation
) {

    public VisionEstimate {
        tagIds = tagIds.clone();
    }

    public int tagCount() {
        return tagIds.length;
    }

    @Override
    public int[] tagIds() {
        return tagIds.clone();
    }

    public Pose2d getPose2d() {
        return pose.toPose2d();
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (!(other instanceof VisionEstimate that)) {
            return false;
        }

        return Arrays.equals(tagIds, that.tagIds)
            && pose.equals(that.pose)
            && timestamp.equals(that.timestamp)
            && Double.compare(avgTagDistanceMeters, that.avgTagDistanceMeters) == 0
            && Double.compare(avgTagAmbiguity, that.avgTagAmbiguity) == 0
            && Double.compare(tagSpanMeters, that.tagSpanMeters) == 0
            && Double.compare(latencySeconds, that.latencySeconds) == 0
            && useVisionRotation == that.useVisionRotation;
    }

    @Override
    public int hashCode() {
        int result = Arrays.hashCode(tagIds);
        result = 31 * result + pose.hashCode();
        result = 31 * result + timestamp.hashCode();
        result = 31 * result + Double.hashCode(avgTagDistanceMeters);
        result = 31 * result + Double.hashCode(avgTagAmbiguity);
        result = 31 * result + Double.hashCode(tagSpanMeters);
        result = 31 * result + Double.hashCode(latencySeconds);
        result = 31 * result + Boolean.hashCode(useVisionRotation);
        return result;
    }

    @Override
    public String toString() {
        return "VisionEstimate["
            + "tagIds=" + Arrays.toString(tagIds)
            + ", pose=" + pose
            + ", timestamp=" + timestamp
            + ", averageTagDistanceMeters=" + avgTagDistanceMeters
            + ", averageTagAmbiguity=" + avgTagAmbiguity
            + ", tagSpanMeters=" + tagSpanMeters
            + ", latencySeconds=" + latencySeconds
            + ", useVisionRotation=" + useVisionRotation
            + ']';
    }
}
