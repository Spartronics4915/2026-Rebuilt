package com.spartronics4915.frc2026.util.vision;

import java.util.Collection;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.ConcurrentSkipListMap;

/**
 * A thread-safe buffer of timestamped samples that supports linear interpolation
 * between entries and range queries.
 *
 * @param <T> Sample type. Must be interpolatable via the supplied
 *            {@link Interpolator}, or use the pre-built factories for
 *            {@code Double} and {@code Pose2d}.
 */
public class ConcurrentTimeBuffer<T> {

    @FunctionalInterface
    public interface Interpolator<T> {
        T interpolate(T a, T b, double t);
    }

    private final ConcurrentSkipListMap<Double, T> buffer = new ConcurrentSkipListMap<>();
    private final Interpolator<T> interpolator;
    private final double historySeconds;

    private ConcurrentTimeBuffer(Interpolator<T> interpolator, double historySeconds) {
        this.interpolator = interpolator;
        this.historySeconds = historySeconds;
    }

    public static ConcurrentTimeBuffer<Double> createDoubleBuffer(double historySeconds) {
        return new ConcurrentTimeBuffer<>(
            (a, b, t) -> a + (b - a) * t,
            historySeconds
        );
    }

    public void addSample(double timestamp, T value) {
        buffer.put(timestamp, value);
        pruneOldEntries(timestamp);
    }

    private void pruneOldEntries(double now) {
        double cutoff = now - historySeconds;
        // Remove all entries strictly before the cutoff, but keep at least one
        // so callers can always find a prior sample.
        while (buffer.size() > 1) {
            Double oldest = buffer.firstKey();
            if (oldest < cutoff) {
                buffer.remove(oldest);
            } else {
                break;
            }
        }
    }

    /**
     * Returns the interpolated value at {@code timestamp}, or the nearest
     * available sample if {@code timestamp} is outside the buffer range.
     */
    public Optional<T> getSample(double timestamp) {
        if (buffer.isEmpty()) return Optional.empty();

        // Exact hit
        T exact = buffer.get(timestamp);
        if (exact != null) return Optional.of(exact);

        Map.Entry<Double, T> floor = buffer.floorEntry(timestamp);
        Map.Entry<Double, T> ceil = buffer.ceilingEntry(timestamp);

        if (floor == null) return Optional.of(ceil.getValue());
        if (ceil == null)  return Optional.of(floor.getValue());

        double t = (timestamp - floor.getKey()) / (ceil.getKey() - floor.getKey());
        return Optional.of(interpolator.interpolate(floor.getValue(), ceil.getValue(), t));
    }

    /** Returns the most recent entry, or {@code null} if empty. */
    public Map.Entry<Double, T> getLatest() {
        return buffer.isEmpty() ? null : buffer.lastEntry();
    }

    /**
     * Returns the maximum absolute value in the range [{@code minTime}, {@code maxTime}].
     *
     * @return the max absolute value in the window, or the nearest bracketing
     *         sample if the window is empty, or {@link OptionalDouble#empty()} only
     *         if the buffer contains no entries at all.
     */
    public OptionalDouble getMaxAbsValueInRange(double minTime, double maxTime) {
        Collection<T> sub = buffer.subMap(minTime, true, maxTime, true).values();
        OptionalDouble inRange = sub.stream()
            .mapToDouble(v -> Math.abs((Double)(Object) v))
            .max();
        if (inRange.isPresent()) return inRange;

        if (buffer.isEmpty()) return OptionalDouble.empty();

        double best = Double.POSITIVE_INFINITY;
        Map.Entry<Double, T> floor = buffer.floorEntry(minTime);
        Map.Entry<Double, T> ceil  = buffer.ceilingEntry(maxTime);
        
        if (floor != null) best = Math.min(best, Math.abs((Double)(Object) floor.getValue()));
        if (ceil != null) best = Math.min(best, Math.abs((Double)(Object) ceil.getValue()));
        return OptionalDouble.of(best);
    }

    /** Exposes the raw map for advanced queries (read-only intent). */
    public ConcurrentSkipListMap<Double, T> getInternalBuffer() {
        return buffer;
    }
    
}