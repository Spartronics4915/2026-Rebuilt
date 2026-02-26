package com.spartronics4915.frc2026.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a geometric region on a FRC field
 */
@FunctionalInterface
public interface FieldRegion {

    /** @return true if the given field-relative position is inside this region */
    boolean contains(Translation2d position);

    /** Creates a standard Axis-Aligned Bounding Box */
    static FieldRegion rectangle(double minX, double maxX, double minY, double maxY) {
        return pos -> pos.getX() >= minX && pos.getX() <= maxX 
                   && pos.getY() >= minY && pos.getY() <= maxY;
    }

    /** Creates a circular boundary */
    static FieldRegion circle(Translation2d center, double radiusMeters) {
        return pos -> pos.getDistance(center) <= radiusMeters;
    }
    
}
