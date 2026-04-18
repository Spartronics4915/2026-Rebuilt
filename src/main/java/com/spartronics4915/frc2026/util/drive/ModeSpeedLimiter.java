package com.spartronics4915.frc2026.util.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Encapsulates speed limiting for a single drive mode.
 * Manages slew rate limiters for X, Y, and omega axes with mode-specific max speeds.
 */
public class ModeSpeedLimiter {
    private final SlewRateLimiter magLimiter;
    private final SlewRateLimiter omegaLimiter;
    private final double maxSpeed;
    private final double maxOmega;

    public ModeSpeedLimiter(
        double maxSpeed, 
        double maxOmega, 
        double timeConstant, 
        double linearSpeed,
        double angularSpeed
    ) {
        this.maxSpeed = maxSpeed;
        this.maxOmega = maxOmega;
        // Apply slew limiting to the translational speed magnitude so that
        // acceleration/deceleration limits are direction-agnostic.
        this.magLimiter = new SlewRateLimiter(maxSpeed * 10, -(maxSpeed / timeConstant), linearSpeed);
        this.omegaLimiter = new SlewRateLimiter(maxOmega * 10, -(maxOmega / timeConstant), angularSpeed);
    }

    /**
     * Resets all limiters to the provided velocity values.
     */
    public void resetAll(double vX, double vY, double omega) {
        double mag = Math.hypot(vX, vY);
        magLimiter.reset(mag);
        omegaLimiter.reset(omega);
    }

    /**
     * Applies clamping and slew rate limiting to the provided velocities.
     * 
     * @param vX X velocity
     * @param vY Y velocity
     * @param omega Angular velocity
     * @return Array containing [limited_vX, limited_vY, limited_omega]
     */
    public double[] limit(double vX, double vY, double omega) {
        // Clamp inputs to configured maxes
        vX = clampSymmetric(vX, maxSpeed);
        vY = clampSymmetric(vY, maxSpeed);
        omega = clampSymmetric(omega, maxOmega);

        // Convert translational input to polar (magnitude + angle)
        double requestedMag = Math.hypot(vX, vY);
        double angle = Math.atan2(vY, vX);

        // Apply slew limiting to the magnitude (direction is preserved)
        double limitedMag = magLimiter.calculate(requestedMag);

        double limitedVX = limitedMag * Math.cos(angle);
        double limitedVY = limitedMag * Math.sin(angle);

        return new double[] {
            limitedVX,
            limitedVY,
            omegaLimiter.calculate(omega)
        };
    }

    private static double clampSymmetric(double value, double limit) {
        return MathUtil.clamp(value, -limit, limit);
    }
    
}
