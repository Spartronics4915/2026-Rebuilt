package com.spartronics4915.frc2026.util.general;

import java.lang.reflect.Field;
import java.util.EnumMap;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Patches WPILib Alert internals to use thread-safe collections.
 *
 * <p>WPILib 2026.2.1 has a race condition in {@code Alert$SendableAlerts}:
 * the active-alerts storage uses a {@link java.util.TreeSet} that is written from
 * vendor library threads (e.g. the Phoenix6 odometry thread) and read from the main
 * robot thread during {@code SmartDashboard.updateValues()}. This produces:
 * <pre>
 *   java.lang.IllegalStateException: Accept exceeded fixed size of 0
 *       at Nodes$FixedNodeBuilder.accept ...
 * </pre>
 *
 * <p>This utility replaces each {@code TreeSet} in every registered Alert group
 * with a {@link ConcurrentSkipListSet}, which provides the same sorted ordering
 * but is safe for concurrent read/write access.
 *
 * <p>Call {@link #apply()} once during {@code robotInit()} <em>after</em> all
 * subsystems are constructed (so all Alert groups are already registered).
 *
 * <p>This patch is a workaround for WPILib issue #7308 and can be removed once
 * the team upgrades to WPILib 2026.3.0 or later.
 */
public final class AlertPatch {

    private AlertPatch() {}

    /**
     * Applies the thread-safety patch to all currently registered Alert groups.
     * Safe to call multiple times; already-patched sets are skipped.
     *
     * @return number of Alert groups patched
     */
    public static int apply() {
        int patched = 0;
        try {
            // Access Alert$SendableAlerts.groups (static Map<String, SendableAlerts>)
            Class<?> sendableAlertsClass = Class.forName(
                "edu.wpi.first.wpilibj.Alert$SendableAlerts");
            Field groupsField = sendableAlertsClass.getDeclaredField("groups");
            groupsField.setAccessible(true);

            @SuppressWarnings("unchecked")
            Map<String, Object> groups = (Map<String, Object>) groupsField.get(null);

            // Access SendableAlerts.m_alerts (EnumMap<AlertType, Set<PublishedAlert>>)
            Field mAlertsField = sendableAlertsClass.getDeclaredField("m_alerts");
            mAlertsField.setAccessible(true);

            for (Object sendableAlerts : groups.values()) {
                @SuppressWarnings("unchecked")
                EnumMap<AlertType, Set<Object>> mAlerts =
                    (EnumMap<AlertType, Set<Object>>) mAlertsField.get(sendableAlerts);

                for (AlertType type : AlertType.values()) {
                    Set<Object> existing = mAlerts.get(type);
                    if (existing == null) {
                        // Initialize with an empty ConcurrentSkipListSet
                        mAlerts.put(type, new ConcurrentSkipListSet<>());
                        patched++;
                    } else if (!(existing instanceof ConcurrentSkipListSet)) {
                        // Replace TreeSet with ConcurrentSkipListSet, preserving contents
                        ConcurrentSkipListSet<Object> safe = new ConcurrentSkipListSet<>(existing);
                        mAlerts.put(type, safe);
                        patched++;
                    }
                    // Already a ConcurrentSkipListSet — skip
                }
            }
        } catch (Exception e) {
            System.err.println("[AlertPatch] Failed to patch Alert thread safety: " + e);
            e.printStackTrace();
        }
        return patched;
    }
}