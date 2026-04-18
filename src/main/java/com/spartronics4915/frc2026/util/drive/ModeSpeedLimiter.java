package com.spartronics4915.frc2026.util.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Encapsulates speed limiting for a single drive mode.
 * Manages slew rate limiters for X, Y, and omega axes with mode-specific max speeds.
 */
public class ModeSpeedLimiter {
    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
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
        this.xLimiter = new SlewRateLimiter(maxSpeed * 10, -(maxSpeed / timeConstant), linearSpeed);
        this.yLimiter = new SlewRateLimiter(maxSpeed * 10, -(maxSpeed / timeConstant), linearSpeed);
        this.omegaLimiter = new SlewRateLimiter(maxSpeed * 10, -(maxSpeed / timeConstant), angularSpeed);
    }

    /**
     * Resets all limiters to the provided velocity values.
     */
    public void resetAll(double vX, double vY, double omega) {
        xLimiter.reset(vX);
        yLimiter.reset(vY);
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
        vX = clampSymmetric(vX, maxSpeed);
        vY = clampSymmetric(vY, maxSpeed);
        omega = clampSymmetric(omega, maxOmega);

        return new double[] {
            xLimiter.calculate(vX),
            yLimiter.calculate(vY),
            omegaLimiter.calculate(omega)
        };
    }

    private static double clampSymmetric(double value, double limit) {
        return MathUtil.clamp(value, -limit, limit);
    }
    
}
