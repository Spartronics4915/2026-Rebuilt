package com.spartronics4915.frc2026.subsystems.vision.results;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface ResultInterface {

    /**
     * Gets the name of the camera that produced this result
     * @return the camera name
     */
    String getCameraName();
    
    /**
     * Gets the timestamp when this result was captured
     * @return timestamp in seconds
     */
    double getTimestampSeconds();
    
    /**
     * Gets the latency of this result
     * @return latency in milliseconds
     */
    double getLatencyMs();
    
    /**
     * Gets the list of targets detected in this result
     * @return list of PhotonTrackedTargets
     */
    List<PhotonTrackedTarget> getTargets();

    /**
     * Gets the average distance to targets
     * @return average distance
     */
    double getAverageDistanceToTargets();
    
    /**
     * Gets the number of targets detected
     * @return target count
     */
    int getTargetCount();
    
    /**
     * Checks if this result has valid data
     * @return true if the result contains valid detection data
     */
    boolean hasValidData();

    /**
     * Gets the the ambiguity
     * @return ambiguity (0 - 1)
     */
    default double getAmbiguity() {return 0.0;};

    /**
     * Checks if this result has a pose
     * @return true if there is a pose present, false otherwise
     */
    default boolean hasPose() {return false;};
}
