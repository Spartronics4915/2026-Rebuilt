package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;
import java.util.function.Supplier;

import static com.spartronics4915.frc2026.Constants.VisionConstants.CAMERA_LOOP_PERIOD_SECONDS;

import com.spartronics4915.frc2026.util.vision.VisionEstimate;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/** Common camera I/O layer. Hardware-specific classes produce timestamped estimates. */
public abstract class CameraIO implements Sendable {

    protected final CameraConfig config;
    private CameraPipeline pipeline = CameraPipeline.defaultPipeline();
    private boolean enabled = true;

    private final List<VisionEstimate> pendingEstimates = new ArrayList<>();
    private final Notifier notifier = new Notifier(this::update);

    protected CameraIO(CameraConfig config) {
        this.config = config;
        notifier.setName("Vision/" + config.name);
    }

    public final void setPipeline(CameraPipeline pipeline) {
        this.pipeline = pipeline;
        applyPipeline(pipeline);
    }

    protected abstract void applyPipeline(CameraPipeline pipeline);

    /** Reads every new measurement currently available from the camera backend. */
    protected abstract List<VisionEstimate> readEstimates();

    public final synchronized void update() {
        if (!enabled) {
            return;
        }

        List<VisionEstimate> estimates = readEstimates();
        if (!estimates.isEmpty()) {
            pendingEstimates.addAll(estimates);
        }
    }

    /** Drains every unprocessed estimate, preserving timestamped measurements. */
    public final synchronized List<VisionEstimate> consumeEstimates() {
        if (pendingEstimates.isEmpty()) {
            return List.of();
        }

        List<VisionEstimate> estimates = List.copyOf(pendingEstimates);
        pendingEstimates.clear();
        return estimates;
    }

    public final void start() {
        notifier.startPeriodic(CAMERA_LOOP_PERIOD_SECONDS);
    }

    public final void stop() {
        notifier.stop();
    }

    public final synchronized void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            pendingEstimates.clear();
        }
    }

    public final synchronized boolean isEnabled() {
        return enabled;
    }

    public final String getName() {
        return config.name;
    }

    public final Transform3d getRobotToCameraTransform() {
        return config.getCurrentTransform();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty(config.name + "/Pipeline", pipeline::name, null);
        builder.addIntegerProperty(config.name + "/Index", pipeline::index, null);
        builder.addStringProperty(config.name + "/Name", () -> config.name, null);
        builder.addBooleanProperty(config.name + "/Dynamic", config::isDynamic, null);
        builder.addBooleanProperty(config.name + "/Enabled", this::isEnabled, this::setEnabled);
    }

    public static class CameraConfig {
        public final String name;
        public final Transform3d transform;
        private final Supplier<Transform3d> dynamicTransformSupplier;
        private final Function<Double, Transform3d> timestampedTransformSupplier;

        public CameraConfig(String name, Transform3d transform) {
            this(name, transform, null, null);
        }

        public CameraConfig(
            String name,
            Transform3d transform,
            Supplier<Transform3d> dynamicTransformSupplier
        ) {
            this(name, transform, dynamicTransformSupplier, null);
        }

        private CameraConfig(
            String name,
            Transform3d transform,
            Supplier<Transform3d> dynamicTransformSupplier,
            Function<Double, Transform3d> timestampedTransformSupplier
        ) {
            this.name = name;
            this.transform = transform;
            this.dynamicTransformSupplier = dynamicTransformSupplier;
            this.timestampedTransformSupplier = timestampedTransformSupplier;
        }

        public static CameraConfig dynamic(
            String name,
            Transform3d initialTransform,
            Supplier<Transform3d> transformSupplier
        ) {
            return new CameraConfig(name, initialTransform, transformSupplier, null);
        }

        public static CameraConfig dynamicAtTimestamp(
            String name,
            Transform3d initialTransform,
            Function<Double, Transform3d> transformSupplier
        ) {
            return new CameraConfig(name, initialTransform, null, transformSupplier);
        }

        /**
         * Turreted camera whose angle can be recovered at the measurement timestamp.
         *
         * <p>The turret-to-camera transform must be defined relative to the turret pivot, and
         * robot-to-turret must locate that pivot in the robot coordinate frame.
         */
        public static CameraConfig turreted(
            String name,
            Transform3d robotToTurret,
            Transform3d turretToCamera,
            DoubleUnaryOperator turretYawDegreesAtTimestamp
        ) {
            Transform3d initialTransform = composeTurretTransform(
                robotToTurret,
                turretToCamera,
                turretYawDegreesAtTimestamp.applyAsDouble(Timer.getFPGATimestamp()));

            return dynamicAtTimestamp(
                name,
                initialTransform,
                timestamp -> composeTurretTransform(
                    robotToTurret,
                    turretToCamera,
                    turretYawDegreesAtTimestamp.applyAsDouble(timestamp)));
        }

        private static Transform3d composeTurretTransform(
            Transform3d robotToTurret,
            Transform3d turretToCamera,
            double turretYawDegrees
        ) {
            Transform3d turretRotation = new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0.0, 0.0, Math.toRadians(turretYawDegrees)));
            return robotToTurret.plus(turretRotation.plus(turretToCamera));
        }

        public boolean isDynamic() {
            return dynamicTransformSupplier != null || timestampedTransformSupplier != null;
        }

        public boolean isTimestampedDynamic() {
            return timestampedTransformSupplier != null;
        }

        public Transform3d getCurrentTransform() {
            if (dynamicTransformSupplier != null) {
                return dynamicTransformSupplier.get();
            }
            if (timestampedTransformSupplier != null) {
                return timestampedTransformSupplier.apply(Timer.getFPGATimestamp());
            }
            return transform;
        }

        public Transform3d getTransformAtTimestamp(double timestampSeconds) {
            if (timestampedTransformSupplier != null) {
                return timestampedTransformSupplier.apply(timestampSeconds);
            }
            return getCurrentTransform();
        }
    }

    public record CameraPipeline(String name, int index) {
        public static CameraPipeline defaultPipeline() {
            return new CameraPipeline("default", -1);
        }
    }
}