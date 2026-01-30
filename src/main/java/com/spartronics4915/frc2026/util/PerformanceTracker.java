package com.spartronics4915.frc2026.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PerformanceTracker {
    
    private final Map<String, TimingStats> timings;
    private final double warningThresholdMs;
    
    private String currentOperation;
    private double operationStartTime;

    public PerformanceTracker(double warningThresholdMs) {
        this.timings = new HashMap<>();
        this.warningThresholdMs = warningThresholdMs;
    }

    public void startTiming(String operationName) {
        this.currentOperation = operationName;
        this.operationStartTime = Timer.getFPGATimestamp();
    }

    public void stopTiming() {
        if (currentOperation == null) return;

        double elapsed = (Timer.getFPGATimestamp() - operationStartTime) * 1000.0; // Convert to ms
        
        TimingStats stats = timings.computeIfAbsent(currentOperation, k -> new TimingStats());
        stats.addSample(elapsed);

        if (elapsed > warningThresholdMs) {
            System.err.println(String.format(
                "WARNING: %s took %.2fms (threshold: %.2fms)",
                currentOperation, elapsed, warningThresholdMs
            ));
        }

        currentOperation = null;
    }

    public void recordTime(String operationName, double timeMs) {
        TimingStats stats = timings.computeIfAbsent(operationName, k -> new TimingStats());
        stats.addSample(timeMs);
    }

    public double getAverageTime(String operationName) {
        TimingStats stats = timings.get(operationName);
        return stats != null ? stats.getAverage() : 0.0;
    }

    public double getPeakTime(String operationName) {
        TimingStats stats = timings.get(operationName);
        return stats != null ? stats.getPeak() : 0.0;
    }

    public void publishMetrics() {
        for (Map.Entry<String, TimingStats> entry : timings.entrySet()) {
            String name = entry.getKey();
            TimingStats stats = entry.getValue();
            
            SmartDashboard.putNumber("Vision/" + name + "/Avg(ms)", stats.getAverage());
            SmartDashboard.putNumber("Vision/" + name + "/Peak(ms)", stats.getPeak());
            SmartDashboard.putNumber("Vision/" + name + "/Last(ms)", stats.getLast());
        }
    }

    public void reset() {
        for (TimingStats stats : timings.values()) {
            stats.reset();
        }
    }

    public boolean isWithinBudget(double budgetMs) {
        TimingStats periodicStats = timings.get("periodic_total");
        return periodicStats == null || periodicStats.getAverage() <= budgetMs;
    }

    private static class TimingStats {
        private double sum = 0.0;
        private int count = 0;
        private double peak = 0.0;
        private double last = 0.0;
        
        private static final double alpha = 0.1;
        private double ema = 0.0;

        void addSample(double value) {
            sum += value;
            count++;
            last = value;
            
            if (value > peak) peak = value;
            
            if (count == 1) ema = value;
                else ema = alpha * value + (1.0 - alpha) * ema;
        }

        double getAverage() {
            return count > 0 ? sum / count : 0.0;
        }

        double getPeak() {
            return peak;
        }

        double getLast() {
            return last;
        }

        void reset() {
            sum = 0.0;
            count = 0;
            peak = 0.0;
            last = 0.0;
            ema = 0.0;
        }
    }
}
