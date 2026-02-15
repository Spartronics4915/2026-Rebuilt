package com.spartronics4915.frc2026.util;

import edu.wpi.first.wpilibj.Timer;

public class TimeVarianceAuthority {
    double prevTime;

    public TimeVarianceAuthority() {
        prevTime = Timer.getFPGATimestamp();
    }

    public double update() {
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - prevTime;
        prevTime = currentTime;
        return deltaTime;
    }
}
