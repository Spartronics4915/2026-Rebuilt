package com.spartronics4915.frc2026.util.logging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.struct.Struct;

/** Small typed facade for subsystem-owned telemetry. */
public final class Telemetry {
    private static final EpilogueBackend ROOT =
        new NTEpilogueBackend(NetworkTableInstance.getDefault()).getNested("/Telemetry");
    private static volatile Logged.Importance minimumImportance = Logged.Importance.DEBUG;

    private Telemetry() {}

    public static Scope scope(String path) {
        return new Scope(ROOT.getNested(path));
    }

    public static void setMinimumImportance(Logged.Importance importance) {
        minimumImportance = importance;
    }

    public static final class Scope {
        public final Writer critical;
        public final Writer info;
        public final Writer debug;
        private final EpilogueBackend backend;

        private Scope(EpilogueBackend backend) {
            this.backend = backend;
            critical = new Writer(backend, Logged.Importance.CRITICAL);
            info = new Writer(backend, Logged.Importance.INFO);
            debug = new Writer(backend, Logged.Importance.DEBUG);
        }

        public Scope child(String path) {
            return new Scope(backend.getNested(path));
        }
    }

    public static final class Writer {
        private final EpilogueBackend backend;
        private final Logged.Importance importance;

        private Writer(EpilogueBackend backend, Logged.Importance importance) {
            this.backend = backend;
            this.importance = importance;
        }

        private boolean enabled() {
            return importance.compareTo(minimumImportance) >= 0;
        }

        public void log(String key, int value) { if (enabled()) backend.log(key, value); }
        public void log(String key, long value) { if (enabled()) backend.log(key, value); }
        public void log(String key, float value) { if (enabled()) backend.log(key, value); }
        public void log(String key, double value) { if (enabled()) backend.log(key, value); }
        public void log(String key, boolean value) { if (enabled()) backend.log(key, value); }
        public void log(String key, String value) { if (enabled()) backend.log(key, value); }
        public void log(String key, Enum<?> value) { if (enabled()) backend.log(key, value); }
        public void log(String key, byte[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, int[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, long[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, float[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, double[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, boolean[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, String[] value) { if (enabled()) backend.log(key, value); }
        public void log(String key, Pose2d value) { log(key, value, Pose2d.struct); }
        public void log(String key, Pose3d value) { log(key, value, Pose3d.struct); }
        public void log(String key, ChassisSpeeds value) { log(key, value, ChassisSpeeds.struct); }
        public void log(String key, Pose2d[] value) { log(key, value, Pose2d.struct); }
        public void log(String key, Pose3d[] value) { log(key, value, Pose3d.struct); }
        public void log(String key, SwerveModuleState[] value) {
            log(key, value, SwerveModuleState.struct);
        }
        public void log(String key, SwerveModulePosition[] value) {
            log(key, value, SwerveModulePosition.struct);
        }

        public <S> void log(String key, S value, Struct<S> struct) {
            if (enabled()) backend.log(key, value, struct);
        }

        public <S> void log(String key, S[] value, Struct<S> struct) {
            if (enabled()) backend.log(key, value, struct);
        }
    }
}
