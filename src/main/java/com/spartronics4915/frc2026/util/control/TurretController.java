package com.spartronics4915.frc2026.util.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Converts a field-relative yaw into an optimal robot-relative turret setpoint.
 * Handles physical limits, path selection hysteresis, and sensor deadbands.
 */
public class TurretController {

    private final double minLimit;
    private final double maxLimit;
    private final double flipDeadband;
    private final double deadband;
    private final Rotation2d mountingOffset;

    private double lastSetpointDeg;
    private boolean onFlippedPath = false;

    /**
     * @param minLimit Physical lower bound (degrees)
     * @param maxLimit Physical upper bound (degrees)
     * @param flipDeadband Degrees required to switch rotation paths to prevent chatter
     * @param deadband Minimum change (degrees) needed to update the setpoint
     * @param mountingOffset Rotational offset between turret zero and robot forward
     */
    public TurretController(
        double minLimit,
        double maxLimit,
        double flipDeadband,
        double deadband,
        Rotation2d mountingOffset
    ) {
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        this.flipDeadband = flipDeadband;
        this.deadband = deadband;
        this.mountingOffset = mountingOffset;
    }

    /**
     * Resets the internal state of the controller
     */
    public void reset(Rotation2d currentPosition) {
        this.lastSetpointDeg = currentPosition.getDegrees();
        this.onFlippedPath = false;
    }

    /**
     * Computes the optimal turret setpoint.
     *
     * @param fieldYaw Target heading in field coordinates
     * @param robotHeading Current robot heading from odometry
     * @param currentPosition Current turret encoder position
     * @return Optimal turret setpoint
     */
    public Rotation2d calculate(Rotation2d fieldYaw, Rotation2d robotHeading, Rotation2d currentPosition) {
        // Calculate the ideal target relative to the robot chassis
        double targetDeg = fieldYaw.minus(robotHeading).minus(mountingOffset).getDegrees();

        // Generate the two possible rotation paths
        double currentDeg = currentPosition.getDegrees();
        double delta = Math.IEEEremainder(targetDeg - currentDeg, 360.0);
        
        double shortest = currentDeg + delta;
        double flipped = shortest - Math.signum(delta) * 360.0;

        boolean shortestSafe = isWithinLimits(shortest);
        boolean flippedSafe = isWithinLimits(flipped);

        double candidate;

        // Path Selection Logic
        if (shortestSafe && flippedSafe) {
            // Both paths are physically possible; use hysteresis to decide
            double distToShortest = Math.abs(shortest - lastSetpointDeg);
            double distToFlipped = Math.abs(flipped - lastSetpointDeg);

            if (onFlippedPath) {
                // Stay flipped unless the shortest path is significantly better
                if (distToShortest < distToFlipped - flipDeadband) {
                    onFlippedPath = false;
                }
            } else {
                // Stay on shortest unless flipped path is significantly better
                if (distToFlipped < distToShortest - flipDeadband) {
                    onFlippedPath = true;
                }
            }
            candidate = onFlippedPath ? flipped : shortest;

        } else if (shortestSafe) {
            onFlippedPath = false;
            candidate = shortest;
        } else if (flippedSafe) {
            onFlippedPath = true;
            candidate = flipped;
        } else {
            boolean nearMax = Math.abs(currentDeg - maxLimit) < Math.abs(currentDeg - minLimit);
            onFlippedPath = nearMax;
            candidate = MathUtil.clamp(onFlippedPath ? flipped : shortest, minLimit, maxLimit);
        }

        // Apply Deadband to suppress micro-corrections
        if (Math.abs(candidate - lastSetpointDeg) < deadband) {
            return Rotation2d.fromDegrees(lastSetpointDeg);
        }

        lastSetpointDeg = candidate;
        return Rotation2d.fromDegrees(candidate);
    }

    public double getLastSetpoint() {
        return lastSetpointDeg;
    }

    public boolean isOnFlippedPath() {
        return onFlippedPath;
    }

    private boolean isWithinLimits(double angle) {
        return angle >= minLimit && angle <= maxLimit;
    }
    
}