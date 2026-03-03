package com.spartronics4915.frc2026.util.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Converts a field-relative yaw from a  into an optimal
 * robot-relative turret setpoint, handling:
 * <ul>
 *   <li>Physical hard limits</li>
 *   <li>Path selection (shortest vs. flipped) with bidirectional hysteresis
 *       to prevent chattering at the crossover boundary</li>
 *   <li>Dead band to suppress micro-corrections from sensor noise</li>
 *   <li>Turret mounting offset baked in so callers don't manage it</li>
 * </ul>
 *
 * <p>Intended usage in {@code AutoAimController}:
 * <pre>
 *   turret.setSetpoint(rotationController.calculate(result.yaw(), swerve.getRelativePose().getRotation(), turret.getPosition()));
 * </pre>
 */
public class TurretController {

    private final double minLimit;
    private final double maxLimit;
    private final double flipDeadband;
    private final double deadband;
    private final Rotation2d mountingOffset;

    private double lastSetpointDeg = 0.0;
    private boolean onFlippedPath = false;

    /**
     * @param minLimit Physical lower bound of the turret
     * @param maxLimit Physical upper bound of the turret
     * @param hysteresis Margin required to switch rotation paths
     *                   The current path is kept unless the other path beats it
     *                   by this amount, in either direction
     * @param deadBand Minimum change needed to update the setpoint
     *                 suppresses micro-corrections from sensor noise
     * @param mountingOffset Fixed rotational offset between the turret's zero and the
     *                       robot's forward direction
     */
    public TurretController(
        double minLimit,
        double maxLimit,
        double hysteresis,
        double deadBand,
        Rotation2d mountingOffset
    ) {
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
        this.flipDeadband = hysteresis;
        this.deadband = deadBand;
        this.mountingOffset = mountingOffset;
    }

    /**
     * Computes the optimal turret setpoint for this tick.
     *
     * @param fieldYaw Field-relative yaw from {@link AutoAim.AutoAimResult#yaw()}
     * @param robotHeading Current robot heading from the swerve odometry
     * @param currentPosition Current turret position as reported by the turret encoder
     * @return Optimal turret setpoint — pass directly to {@code turret.setSetpoint()}
     */
    public Rotation2d calculate(Rotation2d fieldYaw, Rotation2d robotHeading, Rotation2d currentPosition) {
        double targetDeg = fieldYaw.minus(robotHeading).minus(mountingOffset).getDegrees();

        double delta = Math.IEEEremainder(targetDeg - currentPosition.getDegrees(), 360.0);
        double shortest = currentPosition.getDegrees() + delta;
        double flipped = shortest - Math.signum(delta) * 360.0;

        boolean shortestSafe = isWithinLimits(shortest);
        boolean flippedSafe = isWithinLimits(flipped);

        double candidate;

        if (!shortestSafe && !flippedSafe) {
            onFlippedPath = false;
            candidate = MathUtil.clamp(shortest, minLimit, maxLimit);
        } else if (shortestSafe && flippedSafe) {
            double shortestDist = Math.abs(shortest - lastSetpointDeg);
            double flippedDist  = Math.abs(flipped  - lastSetpointDeg);
            onFlippedPath = onFlippedPath
                ? shortestDist + flipDeadband >= flippedDist
                : flippedDist + flipDeadband < shortestDist;
            candidate = onFlippedPath ? flipped : shortest;
        } else {
            onFlippedPath = flippedSafe;
            candidate = flippedSafe ? flipped : shortest;
        }

        if (Math.abs(candidate - lastSetpointDeg) < deadband) {
            return Rotation2d.fromDegrees(lastSetpointDeg);
        }

        lastSetpointDeg = candidate;
        return Rotation2d.fromDegrees(candidate);
    }

    /** @return The last setpoint commanded by {@link #calculate}, in degrees. */
    public double getLastSetpoint() {
        return lastSetpointDeg;
    }

    /** @return Whether the controller is currently routing via the flipped path. */
    public boolean isOnFlippedPath() {
        return onFlippedPath;
    }

    /** @return Whether the turret is currently within its limits. */
    private boolean isWithinLimits(double angle) {
        return angle >= minLimit && angle <= maxLimit;
    }
    
}