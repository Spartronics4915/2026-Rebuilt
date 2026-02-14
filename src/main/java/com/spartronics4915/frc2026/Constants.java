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

import java.util.Map;

import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.PhotonProcessor;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

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
        public static final double TILT_THRESHOLD_DEGREES = 5.0;
        public static final double TILT_DEBOUNCE = 0.2;

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
                SIM_CAMERA_PROPERTIES.setCalibration(1600, 1200, Rotation2d.fromDegrees(97.65));
                SIM_CAMERA_PROPERTIES.setCalibError(0.64, 0.02);
                SIM_CAMERA_PROPERTIES.setFPS(80);
                SIM_CAMERA_PROPERTIES.setAvgLatencyMs(40);
                SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(5);
            }

        public static final class StdDevConstants {
            public static final double baseXYStdDev = 0.08;
            public static final double baseThetaStdDev = 0.08;
            public static final double distanceWeight = 0.8;
            public static final double ambiguityWeight = 0.8;
            public static final double areaWeight = 0.6;
            public static final double anisotropyWeight = 0.6;
            public static final double motionWeight = 0.4;
            public static final double latencyWeight = 0.4;
        }

        public static final class CameraConstants {

            public static final Transform3d RIGHT_CAMERA_TRANSFORM = new Transform3d(
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

            public static final Transform3d LEFT_CAMERA_TRANSFORM =  new Transform3d(
                new Translation3d(
                    0.315, 
                    0, 
                    0.14
                ),
                new Rotation3d(
                    0, 
                    Math.toRadians(-28), 
                    0
                )
            );

            public static final Map<String, ProcessorInterface> cameras = Map.of(
                "right", new PhotonProcessor(
                    "right", 
                    LAYOUT,
                    RIGHT_CAMERA_TRANSFORM, 
                    SIM_CAMERA_PROPERTIES,
                    () -> new ChassisSpeeds()
                ),
                "left", new PhotonProcessor(
                    "left", 
                    LAYOUT,
                    LEFT_CAMERA_TRANSFORM,
                    SIM_CAMERA_PROPERTIES,
                    () -> new ChassisSpeeds()
                )
            );
        }

        public static final double MIN_COSINE_VALUE = 0.01;

    }

    public static final class SuperstructureConstants {

        public static final Transform2d turretTransform = new Transform2d(
            new Translation2d(0, 0),
            new Rotation2d()
        );

    }

    public static class ShooterConstants {
        
        public static final int LEAD_MOTOR_ID = 22;
        public static final int FOLLOWER_MOTOR_ID = 23;

        public static final double P = 0.35;
        public static final double I = 0;
        public static final double D = 0;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double SENSOR_MECHANISM_RATIO = 0.9375;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

    }
    
    public static class HoodConstants {

        public static final int MOTOR_ID = 21;

        public static final double P = 250.0;
        public static final double I = 0.0;
        public static final double D = 0.1;

        public static final double MAX_VELOCITY = 10;
        public static final double MAX_ACCELERATION = 10;

        public static final double DELTA_TIME = 0.02;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double SENSOR_MECHANISM_RATIO = 85.3333333;

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(35);

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
    }

    public static class PivotConstants {

        public static final int MOTOR_ID = 14;

        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final double DELTA_TIME = 0.02;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double SENSOR_MECHANISM_RATIO = 16.875;

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
    }

    public static class FeederConstants {
        
        public static final int MOTOR_ID = 18;

        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 60;
        public static final double LOWER_LIMIT = 40;

        public static final double LOWER_TIME = 1;
        public static final double SENSOR_MECHANISM_RATIO = 20/9;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

    }

    public static class IndexerConstants {
        
        public static final int MOTOR_ID = 17;

        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 60;
        public static final double LOWER_LIMIT = 40;

        public static final double LOWER_TIME = 1;
        public static final double SENSOR_MECHANISM_RATIO = 12;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

    }

}
