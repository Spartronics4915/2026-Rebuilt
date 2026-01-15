// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.Camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class SwerveConstants {
        public static final double TRACK_WIDTH = 22.475 / 12;
        public static final double WHEEL_BASE = 22.475 / 12;
        public static final double CHASSIS_RADIUS = Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);

        public static final double MAX_SPEED = Units.feetToMeters(24);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(MAX_SPEED * Math.PI / CHASSIS_RADIUS);

        public static Rotation2d TELEOP_HEADING_OFFSET = Rotation2d.fromDegrees(0.0);

        public static boolean IS_FIELD_RELATIVE = false;

        public static final double STICK_DEADBAND = 0.1;
    }

    public static final class VisionConstants {
        public static final List<Camera> cameraList = new ArrayList<>();

        public static final SimCameraProperties simCameraProperties = new SimCameraProperties();
            static {
                simCameraProperties.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
                simCameraProperties.setCalibError(0.12, 0.04);
                simCameraProperties.setFPS(60);
                simCameraProperties.setAvgLatencyMs(15);
                simCameraProperties.setLatencyStdDevMs(5);
            }

        public static final AprilTagFieldLayout rebuiltApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
        public enum CameraType {
            LUMA, LIMELIGHT
        }

        public enum VisionState {
            GLOBAL, LOCAL
        }
    }

}
