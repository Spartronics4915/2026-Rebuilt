// Team 2056 controller shaping and slew adaptation; see THIRD_PARTY_NOTICES.md.
package com.spartronics4915.frc2026.util.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Shapes and limits driver requests in the currently selected drive frame. */
public final class TeleopDriveInput {
    private final double maxSpeed;
    private final double maxOmega;
    private final double deadband;
    private final OPRSlewRateLimiter xLimiter;
    private final OPRSlewRateLimiter yLimiter;
    private final OPRSlewRateLimiter rotationLimiter;
    private final ChassisSpeeds output = new ChassisSpeeds();
    private boolean translationRequested;
    private boolean rotationRequested;

    public TeleopDriveInput(double maxSpeed, double maxOmega, double deadband,
            double translationRate, double translationRateRamp,
            double rotationRate, double rotationRateRamp) {
        if (!Double.isFinite(maxSpeed) || maxSpeed <= 0
                || !Double.isFinite(maxOmega) || maxOmega <= 0
                || !Double.isFinite(deadband) || deadband < 0 || deadband >= 1) {
            throw new IllegalArgumentException("Invalid speed limits or deadband");
        }
        this.maxSpeed = maxSpeed;
        this.maxOmega = maxOmega;
        this.deadband = deadband;
        xLimiter = new OPRSlewRateLimiter(translationRate, translationRateRamp);
        yLimiter = new OPRSlewRateLimiter(translationRate, translationRateRamp);
        rotationLimiter = new OPRSlewRateLimiter(rotationRate, rotationRateRamp);
    }

    /** Team 2056's hard deadband (no rescaling), then sign-preserving square. */
    public static double curve(double input, double deadband) {
        if (!Double.isFinite(input)) return 0;
        double bounded = Math.max(-1, Math.min(1, input));
        return Math.abs(bounded) < deadband ? 0 : bounded * Math.abs(bounded);
    }

    /** Returns a reused result; callers must not retain or mutate it. */
    public ChassisSpeeds calculate(double x, double y, double rotation, double dtSeconds) {
        double targetX = curve(x, deadband) * maxSpeed;
        double targetY = curve(y, deadband) * maxSpeed;
        double targetOmega = curve(rotation, deadband) * maxOmega;
        // Bound the target before limiting to avoid a diagonal request above max speed.
        double magnitude = Math.hypot(targetX, targetY);
        if (magnitude > maxSpeed) {
            targetX *= maxSpeed / magnitude;
            targetY *= maxSpeed / magnitude;
        }
        output.vxMetersPerSecond = xLimiter.calculate(targetX, dtSeconds);
        output.vyMetersPerSecond = yLimiter.calculate(targetY, dtSeconds);
        output.omegaRadiansPerSecond = rotationLimiter.calculate(targetOmega, dtSeconds);
        // Independent axis ramps can transiently exceed the circular speed limit.
        // Like Team 2056's drive output, cap the final translation vector as well.
        double outputMagnitude = Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond);
        if (outputMagnitude > maxSpeed) {
            output.vxMetersPerSecond *= maxSpeed / outputMagnitude;
            output.vyMetersPerSecond *= maxSpeed / outputMagnitude;
        }
        translationRequested = targetX != 0 || targetY != 0
            || Math.hypot(output.vxMetersPerSecond, output.vyMetersPerSecond) > 1e-6;
        rotationRequested = targetOmega != 0 || Math.abs(output.omegaRadiansPerSecond) > 1e-6;
        return output;
    }

    public boolean hasTranslationRequest() { return translationRequested; }
    public boolean hasRotationRequest() { return rotationRequested; }

    /** Keep the Y history consistent while the existing trench controller owns Y. */
    public void resetY(double velocity) { yLimiter.reset(velocity); }

    public void reset() {
        xLimiter.reset(0);
        yLimiter.reset(0);
        rotationLimiter.reset(0);
        output.vxMetersPerSecond = 0;
        output.vyMetersPerSecond = 0;
        output.omegaRadiansPerSecond = 0;
        translationRequested = false;
        rotationRequested = false;
    }
}
