// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.Camera;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Luma;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

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
        public static final AprilTagFieldLayout rebuiltApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final List<Camera> cameraList = List.of(
            new Luma(
                "daniil", 
                CameraType.LUMA, 
                rebuiltApriltagFieldLayout, 
                new Transform3d(
                    new Translation3d(0.3556, 0, 0.0508),
                    new Rotation3d(
                        Rotation2d.fromDegrees(0).getRadians(), 
                        Rotation2d.fromDegrees(-10).getRadians(), 
                        Rotation2d.fromDegrees(0).getRadians()
                    )
                )
            ),
            new Luma(
                "evan", 
                CameraType.LUMA, 
                rebuiltApriltagFieldLayout, 
                new Transform3d(
                    new Translation3d(-0.3556, 0, 0.0508),
                    new Rotation3d(
                        Rotation2d.fromDegrees(180).getRadians(), 
                        Rotation2d.fromDegrees(-10).getRadians(), 
                        Rotation2d.fromDegrees(180).getRadians()
                    )
                )
            )
        );

        public static final SimCameraProperties simCameraProperties = new SimCameraProperties();
            static {
                simCameraProperties.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
                simCameraProperties.setCalibError(0.12, 0.04);
                simCameraProperties.setFPS(60);
                simCameraProperties.setAvgLatencyMs(15);
                simCameraProperties.setLatencyStdDevMs(5);
            }

        public static final double baseTransverseMultiTagStdDevs = 0.5;
        public static final double baseAngularMultiTagStdDevs = 0.5;

        public static final double baseTransverseSingleTagStdDevs = 0.5;
        public static final double baseAngularSingleTagStdDevs = 0.5;

        public static final double tagReward = 0;
        public static final double distancePunishment = 0;
        public static final double ambiguityPunishment = 0;
        public static final double transverseVelocityPunishment = 0;
        public static final double angularVelocityPunishment = 0;
        
        public enum CameraType {
            LUMA, LIMELIGHT
        }

        public enum VisionState {
            GLOBAL, LOCAL
        }
    }

}
