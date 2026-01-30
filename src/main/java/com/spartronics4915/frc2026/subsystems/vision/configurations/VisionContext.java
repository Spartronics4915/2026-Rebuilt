package com.spartronics4915.frc2026.subsystems.vision.configurations;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionContext {
    private final AprilTagFieldLayout fieldLayout;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final VisionConfiguration config;
    private VisionState currentState;

    public VisionContext(
        AprilTagFieldLayout fieldLayout,
        Supplier<Pose2d> robotPoseSupplier,
        VisionConfiguration config,
        VisionState initialState
    ) {
        this.fieldLayout = fieldLayout;
        this.robotPoseSupplier = robotPoseSupplier;
        this.config = config;
        this.currentState = initialState;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public Pose2d getCurrentRobotPose() {
        return robotPoseSupplier.get();
    }

    public VisionConfiguration getConfig() {
        return config;
    }

    public VisionState getCurrentState() {
        return currentState;
    }

    public void setState(VisionState state) {
        this.currentState = state;
    }

    public enum VisionState {
        GLOBAL, LOCAL, IDLE
    }
}
