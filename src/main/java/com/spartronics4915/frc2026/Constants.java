// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.PhotonProcessor;
import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.PathplannerConfigs;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class SwerveConstants {
        public static final double MAX_SPEED = 6;
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(8);

        public static Rotation2d TELEOP_HEADING_OFFSET = Rotation2d.fromDegrees(0.0);

        public static boolean IS_FIELD_RELATIVE = true;

        public static final double STICK_DEADBAND = 0.05;

        public enum SwerveConfigurations {
            TEST_CHASSIS("test-chassis", PathplannerConfigs.TEST_CHASSIS),
            COMP_CHASSIS("comp-chassis", PathplannerConfigs.COMP_CHASSIS);

            public String directory;
            public PathplannerConfigs pathplannerConfig;
            private SwerveConfigurations(String directory, PathplannerConfigs pathplannerConfig) {
                this.directory = "swerve/" + directory;
                this.pathplannerConfig = pathplannerConfig;
            }
        }

        public static final class AutoConstants {
            public static final PIDConstants translationPID = new PIDConstants(5.0,0,0);
            public static final PIDConstants rotationPID = new PIDConstants(5.0,0,0);

            public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                AutoConstants.translationPID, 
                AutoConstants.rotationPID
            );

            public static final Translation2d towerPose = new Translation2d(1.061, 3.745);
            public static final Translation2d centerPose = new Translation2d(8.271, 4.035);
            public static final Translation2d hubPose = new Translation2d(4.625, 4.035);
            public static final Translation2d trenchTransform = new Translation2d(0, -3.4);
            public static final Translation2d bumpTransform = new Translation2d(0, -1.523);
            public static final Translation2d approachTransform = new Translation2d(-1.1, 0);
            public static final Translation2d exitTransform = new Translation2d(centerPose.getX() - hubPose.getX(), 0);

            public static final PathConstraints defaultPathConstraints = new PathConstraints(
                4,
                5.0,
                1/2 * Math.PI,
                1 * Math.PI
            );

            public static final PathConstraints trenchPathConstraints = new PathConstraints(
                2.0,
                2.0,
                1/4 * Math.PI,
                1 * Math.PI
            );

            public static final PathConstraints bumpPathConstraints = new PathConstraints(
                0.5,
                0.5,
                1/4 * Math.PI,
                1/2 * Math.PI
            );

            public static final Rotation2d trenchApproachAngle = Rotation2d.fromDegrees(0.0);
            public static final Rotation2d bumpApproachAngle = Rotation2d.fromDegrees(45.0);

            public enum PathplannerConfigs {
                TEST_CHASSIS(new RobotConfig(
                    Pounds.of(15),
                    KilogramSquareMeters.of(3),
                    new ModuleConfig(
                        Inches.of(2),
                        MetersPerSecond.of(5.4),
                        1.916,
                        DCMotor.getKrakenX60(1),
                        6.75,
                        Amps.of(40),
                        1
                    ),
                    new Translation2d(Inches.of(12.634).in(Meter), Inches.of(12.280).in(Meter)), // Front left
                    new Translation2d(Inches.of(12.634).in(Meter), Inches.of(-12.280).in(Meter)), // Front right
                    new Translation2d(Inches.of(-12.634).in(Meter), Inches.of(12.280).in(Meter)), // Back left
                    new Translation2d(Inches.of(-12.634).in(Meter), Inches.of(-12.280).in(Meter))  // Back right
                )),

                COMP_CHASSIS(new RobotConfig(
                    Pounds.of(15),
                    KilogramSquareMeters.of(2),
                    new ModuleConfig(
                        Inches.of(2.0),
                        MetersPerSecond.of(24),
                        1.916,
                        DCMotor.getKrakenX60(1),
                        6.03,
                        Amps.of(60),
                        1
                    ),
                    new Translation2d(Inches.of(14).in(Meter), Inches.of(13).in(Meter)), // Front left
                    new Translation2d(Inches.of(14).in(Meter), Inches.of(-13).in(Meter)), // Front right
                    new Translation2d(Inches.of(-14).in(Meter), Inches.of(13).in(Meter)), // Back left
                    new Translation2d(Inches.of(-14).in(Meter), Inches.of(-13).in(Meter))  // Back right
                ));

                public com.pathplanner.lib.config.RobotConfig config;

                private PathplannerConfigs(RobotConfig config) {
                    this.config = config;
                }
            }
        }
    }

    public static final class VisionConstants {
        public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        
        public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();
            static {
                SIM_CAMERA_PROPERTIES.setCalibration(1280, 800, Rotation2d.fromDegrees(97.65));
                SIM_CAMERA_PROPERTIES.setCalibError(0.84, 0.02);
                SIM_CAMERA_PROPERTIES.setFPS(80);
                SIM_CAMERA_PROPERTIES.setAvgLatencyMs(40);
                SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(5);
            }

        public static Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(
                0.315, 
                0.116,
                0.14
            ),
            new Rotation3d(
                0, 
                Math.toRadians(-28), 
                Math.toRadians(-18)
            )
        );

        public static PhotonProcessor RIGHT_PROCESSOR = new PhotonProcessor(
            "daniil", 
            new PhotonPoseEstimator(LAYOUT, RIGHT_CAMERA_TRANSFORM), 
            100
        );

        public static Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(
                -0.0381, 
                -0.34798,
                0.14605
            ),
            new Rotation3d(
                0, 
                Math.toRadians(-28), 
                Math.toRadians(282)
            )
        );

        public static PhotonProcessor LEFT_PROCESSOR = new PhotonProcessor(
            "evan", 
            new PhotonPoseEstimator(LAYOUT, LEFT_CAMERA_TRANSFORM), 
            100
        );

    }
}
