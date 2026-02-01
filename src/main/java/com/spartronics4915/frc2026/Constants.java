// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.PhotonProcessor;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext.VisionState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class SwerveConstants {
        public static final double MAX_SPEED = 18;
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(24);

        public static Rotation2d TELEOP_HEADING_OFFSET = Rotation2d.fromDegrees(0.0);

        public static boolean IS_FIELD_RELATIVE = false;

        public static final double STICK_DEADBAND = 0.05;
    }

    public static final class VisionConstants {
        public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        
        public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();
            static {
                SIM_CAMERA_PROPERTIES.setCalibration(1280, 800, Rotation2d.fromDegrees(97.65));
                SIM_CAMERA_PROPERTIES.setCalibError(0.84, 0.02);
                SIM_CAMERA_PROPERTIES.setFPS(60);
                SIM_CAMERA_PROPERTIES.setAvgLatencyMs(40);
                SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(10);
            }

        public static Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(
                -0.34798, 
                -0.0381, 
                0.14605
            ),
            new Rotation3d(
                0, 
                Math.toRadians(-28), 
                Math.toRadians(72)
            )
        );

        public static PhotonProcessor RIGHT_PROCESSOR = new PhotonProcessor(
            "evan", 
            new PhotonPoseEstimator(LAYOUT, RIGHT_CAMERA_TRANSFORM), 
            0
        );
    }
}
