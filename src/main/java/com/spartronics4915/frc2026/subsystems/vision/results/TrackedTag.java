package com.spartronics4915.frc2026.subsystems.vision.results;

/**
 * A camera-agnostic representation of a single tracked AprilTag.
 * This class is mutable to allow for object pooling and zero-allocation updates.
 */
public final class TrackedTag {

    private int fiducialId;
    private double area;
    private double ambiguity;

    public TrackedTag(int fiducialId, double area, double ambiguity) {
        set(fiducialId, area, ambiguity);
    }

    public void set(int fiducialId, double area, double ambiguity) {
        this.fiducialId = fiducialId;
        this.area = area;
        this.ambiguity = ambiguity;
    }

    public int getFiducialId() {
        return fiducialId;
    }

    public double getArea() {
        return area;
    }

    public double getAmbiguity() {
        return ambiguity;
    }
    
}