package com.spartronics4915.frc2026.util;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A reusable utility that maps custom enums to physical field regions.
 * Evaluates regions in the order they were added.
 * * @param <Z> The enum representing the specific year's field zones.
 */
public class FieldZoneMap<Z extends Enum<Z>> {
    
    // LinkedHashMap keeps the insertion order, which is important 
    // if zones overlap and you want one to have priority (4481 Rembrandts tech)
    private final Map<Z, FieldRegion> regions = new LinkedHashMap<>();
    private final Z defaultZone;

    public FieldZoneMap(Z defaultZone) {
        this.defaultZone = defaultZone;
    }

    /**
     * Maps an enum value to a physical area on the field.
     */
    public void addZone(Z zone, FieldRegion region) {
        regions.put(zone, region);
    }

    /**
     * Checks the given position against all registered regions.
     * @return The first zone that contains the position, or the default zone.
     */
    public Z evaluate(Translation2d position) {
        for (Map.Entry<Z, FieldRegion> entry : regions.entrySet()) {
            if (entry.getValue().contains(position)) {
                return entry.getKey();
            }
        }
        return defaultZone;
    }
    
}
