// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.PhotonProcessor;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Constants {

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int TESTING_CONTROLLER_PORT = 2;
    }

    //#region General

    public static class GeneralConstants {
        public static final CANBus CAN_BUS = new CANBus("Hydra");
    }  

    //#endregion
    //#region Swerve

    public static final class SwerveConstants {
        public static final double MAX_SPEED = 6;
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(8);

        public static final boolean DEFAULT_IS_FIELD_RELATIVE = true;

        public static final double STICK_DEADBAND = 0.05;
        public static final double TILT_THRESHOLD_DEGREES = 1.0;
        public static final double TILT_DEBOUNCE = 0.3;

        public static final Constraints trenchAlignConstraints = new Constraints(3, 3);

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

        //#region Autos

        public static final class AutoConstants {
            public static final PIDConstants translationPID = new PIDConstants(7.5,0,0);
            public static final PIDConstants rotationPID = new PIDConstants(5.0,0,0);

            public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                AutoConstants.translationPID, 
                AutoConstants.rotationPID
            );

            public static final PIDConstants alignTranslationPID = new PIDConstants(2.0,0,0);
            public static final PIDConstants alignRotationPID = new PIDConstants(2.0,0,0);

            public static final PPHolonomicDriveController autoAlignPIDController = new PPHolonomicDriveController(
                AutoConstants.alignTranslationPID, 
                AutoConstants.alignRotationPID
            );

            public static final double defaultOutpostWaitTime = 3.0;

            public static final Time endTriggerDebounce = Seconds.of(0.04);
            public static final Rotation2d rotationTolerance = Rotation2d.fromDegrees(3.0);
            public static final Distance positionTolerance = Centimeter.of(1.5);
            public static final LinearVelocity speedTolerance = InchesPerSecond.of(2);

            public static final Translation2d towerPose = new Translation2d(1.061, 3.745);
            public static final Translation2d centerPose = new Translation2d(8.271, 4.035);
            public static final Translation2d hubPose = new Translation2d(4.625, 4.035);
            public static final Translation2d outpostPose = new Translation2d(0.0, 0.666);
            public static final Translation2d depotPose = new Translation2d(0.0, 5.964);
            public static final Translation2d fuelZoneXAxisPose = new Translation2d(7.512, 4.035);

            public static final Translation2d towerTransform = new Translation2d(0.0, 0.49075);
            public static final Translation2d trenchTransform = new Translation2d(0, -3.4);
            public static final Translation2d bumpTransform = new Translation2d(0, -1.523);
            public static final Translation2d bumpTrenchDivTransform = new Translation2d(0, 2.604);
            public static final Translation2d approachTransform = new Translation2d(-1.1, 0);
            public static final Translation2d exitTransform = new Translation2d(centerPose.getX() - hubPose.getX(), 0);
            public static final Translation2d fuelIntakeTransform = new Translation2d(0, 2.308);

            public static final Distance robotLength = Millimeters.of(818.5);
            public static final Distance robotWidth = Millimeters.of(875.65);
            public static final Distance intakeLength = Millimeters.of(213.05);
            public static final Distance towerPadding = Inches.of(10);
            public static final Distance centerPadding = Inches.of(2); // Padding away from center so we don't hit opponent robots
            public static final Distance bumperThickness = Millimeters.of(72.7);

            public static final PathConstraints defaultPathConstraints = new PathConstraints(
                3.0,
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

            public static final PathConstraints intakePathConstraints = new PathConstraints(
                2.0,
                5.0,
                1/2 * Math.PI,
                1 * Math.PI
            );

            public static final Rotation2d trenchApproachAngle = Rotation2d.fromDegrees(0.0);
            public static final Rotation2d bumpApproachAngle = Rotation2d.fromDegrees(45.0);
            public static final Rotation2d startOfFuelAngle = Rotation2d.fromDegrees(299); //This guy might need to be changed
            public static final Rotation2d endOfQuadrantAngle = Rotation2d.fromDegrees(-90.0);

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
                    Pounds.of(100),
                    KilogramSquareMeters.of(2),
                    new ModuleConfig(
                        Inches.of(2.0),
                        MetersPerSecond.of(24),
                        2.255,
                        DCMotor.getKrakenX60(1),
                        6.03,
                        Amps.of(60),
                        1
                    ),
                    new Translation2d(Inches.of(12.1640885).in(Meter), Inches.of(9.585892).in(Meter)), // Front left
                    new Translation2d(Inches.of(12.1640885).in(Meter), Inches.of(-9.585892).in(Meter)), // Front right
                    new Translation2d(Inches.of(-12.1640885).in(Meter), Inches.of(9.585892).in(Meter)), // Back left
                    new Translation2d(Inches.of(-12.1640885).in(Meter), Inches.of(-9.585892).in(Meter))  // Back right
                ));

                public com.pathplanner.lib.config.RobotConfig config;

                private PathplannerConfigs(RobotConfig config) {
                    this.config = config;
                }
            }
        }
        //#endregion
    }

    //#endregion
    //#region Vision

    public static final class VisionConstants {
        public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        
        public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();
            static {
                SIM_CAMERA_PROPERTIES.setCalibration(1600, 1200, Rotation2d.fromDegrees(97.65));
                SIM_CAMERA_PROPERTIES.setCalibError(0.05, 0.005);
                SIM_CAMERA_PROPERTIES.setFPS(100);
                SIM_CAMERA_PROPERTIES.setAvgLatencyMs(20);
                SIM_CAMERA_PROPERTIES.setLatencyStdDevMs(0);
            }

        public static final class StdDevConstants {
            public static final double baseXYStdDev = 0.1;  // 0.5 — trust vision closer to odometry levels
            public static final double baseThetaStdDev = 0.2;  // 0.5 — heading from vision is still less reliable
            public static final double distanceWeight = 0.8;  // 1.0 — less aggressive distance penalty
            public static final double ambiguityWeight = 0.6;  // 0.8
            public static final double areaWeight = 0.5;  // 0.6
            public static final double anisotropyWeight = 0.5;  // 0.6
            public static final double motionWeight = 0.3;  // 0.4
            public static final double latencyWeight = 0.3;  // 0.4

            /**
             * Smoothing factor for the exponential moving average applied to distance and area.
             * Range [0.0, 1.0] — lower values smooth more but react slower to real changes.
             * At 20fps: 0.05 - 1s lag, 0.15 - 0.3s lag, 0.30 - 0.15s lag.
             */
            public static final double smoothingAlpha = 0.15;
        }

        public static final class CameraConstants {

            public static final Transform3d LEFT_CAMERA_TRANSFORM = new Transform3d(
                new Translation3d(
                    -0.1272, 
                    0.329413,
                    0.4076
                ),
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-26), 
                    Math.toRadians(70)
                )
            );

            public static final Transform3d RIGHT_CAMERA_TRANSFORM =  new Transform3d(
                new Translation3d(
                    -0.125205, 
                    -0.334776, 
                    0.257945
                ),
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-26), 
                    Math.toRadians(-70)
                )
            );

            public static final Transform3d BACK_CAMERA_TRANSFORM =  new Transform3d(
                new Translation3d(
                    -0.3070, 
                    0.1270, 
                    0.276496
                ),
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-23), 
                    Math.toRadians(180)
                )
            );

            public static final Map<String, ProcessorInterface> cameras = Map.of(
                "evan", new PhotonProcessor(
                    "evan", 
                    LAYOUT,
                    LEFT_CAMERA_TRANSFORM, 
                    new StdDevCalculator(),
                    SIM_CAMERA_PROPERTIES,
                    () -> new ChassisSpeeds()
                ),
                "daniil", new PhotonProcessor(
                    "daniil", 
                    LAYOUT,
                    RIGHT_CAMERA_TRANSFORM,
                    new StdDevCalculator(),
                    SIM_CAMERA_PROPERTIES,
                    () -> new ChassisSpeeds()
                ),
                "val", new PhotonProcessor(
                    "val", 
                    LAYOUT,
                    BACK_CAMERA_TRANSFORM,
                    new StdDevCalculator(),
                    SIM_CAMERA_PROPERTIES,
                    () -> new ChassisSpeeds()
                )
            );
        }

        public static final double MIN_COSINE_VALUE = 0.01;

    }

    //#endregion
    //#region Superstructure

    public static final class SuperstructureConstants {
        
        public static final Translation2d TURRET_TRANSLATION = new Translation2d(-0.1185, -0.1568);

        public static final Rotation2d PIVOT_SAFE_THRESHOLD = Rotation2d.fromDegrees(0);
        public static final Rotation2d TURRET_MIN_SAFE_THRESHOLD = Rotation2d.fromDegrees(0);
        public static final Rotation2d TURRET_MAX_SAFE_THRESHOLD = Rotation2d.fromDegrees(0);

        public static final Distance bumpLength = Inches.of(48.93);
        public static final Distance trenchLength = Inches.of(30);

        public static final double percentLoss = 0.08; // Percent loss on shooter to ball transfer
    }

    //#endregion
    //#region Shooter

    public static class ShooterConstants {
        
        public static final int LEAD_MOTOR_ID = 22;
        public static final int FOLLOWER_MOTOR_ID = 23;

        public static final double P = 0.4;
        public static final double I = 0;
        public static final double D = 0.002;
        public static final double V = 0.123;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 60;
        public static final double LOWER_LIMIT = 40;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 0.9375;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
            .withKV(V);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
    }

    //#endregion
    //#region Hood
    
    public static class HoodConstants {

        public static final int MOTOR_ID = 21;

        public static final double P = 250.0;
        public static final double I = 0.0;
        public static final double D = 0.1;

        public static final double MAX_VELOCITY = 10;
        public static final double MAX_ACCELERATION = 10;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 85.3333333;

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
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
    }

    //#endregion
    //#region Pivot

    public static class PivotConstants {

        public static final int MOTOR_ID = 14;
        public static final int ENCODER_ID = 16;

        public static final double P = 60;
        public static final double I = 0.0;
        public static final double D = 0.2;

        public static final double MAX_VELOCITY = 5;
        public static final double MAX_ACCELERATION = 5;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 50.625;

        public static final double MAGNET_OFFSET = -0.665;
        public static final SensorDirectionValue ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-2);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(120);

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
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
    }

    //#endregion
    //#region Feeder

    public static class FeederConstants {
        
        public static final int MOTOR_ID = 18;

        public static final double P = 0.3;
        public static final double I = 0;
        public static final double D = 0.0003;
        public static final double V = 0.24;

        public static final double MAX_RPS = 100;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 60;
        public static final double LOWER_LIMIT = 40;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 20/9;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
            .withKV(V);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
    }

    //#endregion
    //#region Indexer

    public static class IndexerConstants {
        
        public static final int MOTOR_ID = 17;

        public static final double P = 0.3;
        public static final double I = 0;
        public static final double D = 0.0003;
        public static final double V = 0.462;

        public static final double MAX_RPS = 100;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 60;
        public static final double LOWER_LIMIT = 40;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 4;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
            .withKV(V);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
    }

    //#endregion
    //#region Turret

    public static class TurretConstants{

        public static final int MOTOR_ID = 19;
        public static final int ENCODER_ID = 20;

        public static final double P = 220;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double MAX_VELOCITY = 7.5;
        public static final double MAX_ACCELERATION = 7.5;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 1.0 / ((12.0/38.0) * (18.0/38.0) * (11.0/84.0));
        public static final double ENCODER_MECHANISM_RATIO = 11.0/84.0;
        public static final double MAGNET_OFFSET = -0.125;

        public static final SensorDirectionValue ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

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
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);

    }

    //#endregion
    //#region Intake
    
    public static class IntakeConstants {

        public static final int MOTOR_ID = 15;

        public static final double MAX_RPS = 100;

        public static final double P = 0.08;
        public static final double I = 0;
        public static final double D = 0.00006;
        public static final double V = 0.37;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 3;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
            .withKV(V);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake);
    }

    //#endregion
    //#region Climber

    public static class ClimberConstants{

        public static final int MOTOR_ID = 24;

        public static final double P = 35.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double MAX_VELOCITY = 4.0;
        public static final double MAX_ACCELERATION = 4.0;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 60;
        public static final double LOWER_LIMIT = 40;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 20;

        public static final double MIN_HEIGHT = 0.0;
        public static final double MAX_HEIGHT = 3.5;

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
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    }

    //#endregion

}
