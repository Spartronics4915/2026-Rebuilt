package com.spartronics4915.frc2026.subsystems.vision.results;

/**
 * A camera-agnostic representation of a single tracked AprilTag.
 */
public final class TrackedTag {

    /** The AprilTag fiducial ID. */
    public final int fiducialId;

    /**
     * Fraction of the camera frame covered by this tag (0.0 – 1.0).
     */
    public final double area;

    /**
     * Pose ambiguity score (0.0–1.0, lower is better).
     * For multi-tag or MegaTag2 results this is typically 0.
     */
    public final double ambiguity;

    public TrackedTag(int fiducialId, double area, double ambiguity) {
        this.fiducialId = fiducialId;
        this.area = area;
        this.ambiguity  = ambiguity;
    }
    
}