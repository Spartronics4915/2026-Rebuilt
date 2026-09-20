// Adapted from Team 2056 PublicCodeBank, OPRSlewRateLimiter.h (70a74d0).
// Copyright (c) 2009-2026 FIRST and other WPILib contributors.
// See THIRD_PARTY_NOTICES.md for the source and BSD license.
package com.spartronics4915.frc2026.util.drive;

/**
 * Team 2056's ramped slew allowance: the allowed output rate grows while chasing
 * a target. This is not a strict signed-acceleration/jerk trajectory generator.
 * Input units are arbitrary; rate is units/s and rateRamp is units/s^2.
 */
public final class OPRSlewRateLimiter {
    private final double maxRate;
    private final double rateRamp;
    private double rate;
    private double value;

    public OPRSlewRateLimiter(double maxRate, double rateRamp) {
        if (!Double.isFinite(maxRate) || maxRate <= 0
                || !Double.isFinite(rateRamp) || rateRamp <= 0) {
            throw new IllegalArgumentException("Rate and rate ramp must be finite and positive");
        }
        this.maxRate = maxRate;
        this.rateRamp = rateRamp;
    }

    /** Uses the caller's elapsed time so all drive axes advance together. */
    public double calculate(double input, double dtSeconds) {
        if (!Double.isFinite(input) || !Double.isFinite(dtSeconds) || dtSeconds < 0) {
            throw new IllegalArgumentException("Input and nonnegative elapsed time must be finite");
        }
        if (dtSeconds == 0) return value;
        double error = input - value;
        double maxChange = rate * dtSeconds;
        if (Math.abs(error) > maxChange) {
            value += Math.copySign(maxChange, error);
            rate = Math.min(maxRate, rate + rateRamp * dtSeconds);
        } else {
            value = input;
            // Upstream uses (previous - input) * dt, which can produce a negative
            // allowance and move away from the target. Settled targets restart the ramp.
            rate = 0;
        }
        return value;
    }

    public void reset(double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Value must be finite");
        this.value = value;
        rate = 0;
    }
}

