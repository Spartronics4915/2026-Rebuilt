package com.spartronics4915.frc2026.util.vision;

import java.util.Collection;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.ConcurrentSkipListMap;

/**
 * A thread-safe buffer of timestamped samples that supports linear interpolation
 * between entries and range queries.
 *
 * <p>Backed by a {@link ConcurrentSkipListMap} so reads and writes from different
 * threads (e.g. the CTRE odometry Notifier and the main robot loop) are safe
 * without external locking.
 *
 * <p>Old entries beyond {@code historySeconds} are pruned automatically on every
 * write to bound memory usage.
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
    public java.util.Optional<T> getSample(double timestamp) {
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
     * Returns the maximum absolute value in the half-open range
     * [{@code minTime}, {@code maxTime}), or empty if no samples exist in range.
     */
    public java.util.OptionalDouble getMaxAbsValueInRange(double minTime, double maxTime) {
        Collection<T> sub = buffer.subMap(minTime, true, maxTime, true).values();
        return sub.stream()
            .mapToDouble(v -> Math.abs((Double)(Object) v))
            .max();
    }

    /** Exposes the raw map for advanced queries (read-only intent). */
    public ConcurrentSkipListMap<Double, T> getInternalBuffer() {
        return buffer;
    }
    
}