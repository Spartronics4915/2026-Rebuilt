package com.spartronics4915.frc2026;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.PathplannerConfigs;
import com.spartronics4915.frc2026.subsystems.vision.cameras.PhotonProcessor;
import com.spartronics4915.frc2026.subsystems.vision.cameras.ProcessorInterface;
import com.spartronics4915.frc2026.subsystems.vision.processing.StdDevCalculator;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

public final class Constants {

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int DEBUG_CONTROLLER_PORT = 2;
    }

    //#region General

    public static class GeneralConstants {
        public static final CANBus CAN_BUS = new CANBus("Hydra");
        public static final CANBus DEACTIVATED_CAN_BUS = new CANBus("Deactivated");
    }  

    //#endregion
    //#region Swerve

    public static final class SwerveConstants {

        public static final double maxSpeed = 7.12; // 5.12
        public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(8); // 6
        public static final double maxSpeedWhenShooting = maxSpeed * 0.4;

        public static final boolean defaultFieldRelative = true;

        public static final double stickDeadband = 0.0;
        public static final double tiltThresholdDegrees = 1.0;
        public static final double tiltDebounce = 0.1;

        public static final Constraints trenchAlignConstraints = new Constraints(3, 3);

        public static final double odomUpdateFrequency = 250.0;

        // Odometry process noise — how much we trust wheel/gyro odometry each loop.
        // These must be loose enough that vision measurements can compete. 254 uses
        // 0.3 m / 0.2 rad; at 0.05 m / 0.005 rad vision would contribute < 2% weight
        // at typical FRC tag distances and the robot pose would never update from vision.
        public static final Matrix<N3, N1> normalStdDevs = VecBuilder.fill(0.3, 0.3, 0.2);
        public static final Matrix<N3, N1> slipStdDevs = VecBuilder.fill(2.0, 2.0, 0.5);

        public static final double slipRecoverySeconds = 0.5;
        public static final double slipThresholdRPS = 1.0;
        public static final double minSpeedDetectMPS = 0.5;
        public static final int slipDebounceCycles = 3;

        public static final double headingLockKP = 7.0;
        public static final double headingLockKD = 0.0;

        public record ModuleConfig(
            int steerMotorId, int driveMotorId, int encoderId,
            Angle encoderOffset,
            Distance xPos, Distance yPos,
            boolean invertDrive
        ) {}

        public enum SwerveConfigurations {
            COMP_CHASSIS(
                new SwerveDrivetrainConstants()
                    .withCANBusName("Hydra")
                    .withPigeon2Id(13),
                AutoConstants.PathplannerConfigs.COMP_CHASSIS,
                compChassisFactory(),
                // Front Left
                new ModuleConfig(1, 2, 3,
                    Rotations.of(-0.30615234375),
                    Inches.of(9.585892), Inches.of(12.1640885),
                    false),
                // Front Right
                new ModuleConfig(4, 5, 6,
                    Rotations.of(-0.094970703125),
                    Inches.of(9.585892), Inches.of(-12.1640885),
                    true),
                // Back Left
                new ModuleConfig(7, 8, 9,
                    Rotations.of(0.1591796875),
                    Inches.of(-9.585892), Inches.of(12.1640885),
                    false),
                // Back Right
                new ModuleConfig(10, 11, 12,
                    Rotations.of(0.155517578125),
                    Inches.of(-9.585892), Inches.of(-12.1640885),
                    true)
            );

            public final SwerveDrivetrainConstants drivetrainConstants;
            public final PathplannerConfigs pathplannerConfig;
            public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] modules;

            @SuppressWarnings("unchecked")
            private SwerveConfigurations(
                SwerveDrivetrainConstants drivetrainConstants,
                PathplannerConfigs pathplannerConfig,
                SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> factory,
                ModuleConfig fl, ModuleConfig fr, ModuleConfig bl, ModuleConfig br
            ) {
                this.drivetrainConstants = drivetrainConstants;
                this.pathplannerConfig = pathplannerConfig;
                this.modules = new SwerveModuleConstants[]{
                    factory.createModuleConstants(
                        fl.steerMotorId(), fl.driveMotorId(), fl.encoderId(),
                        fl.encoderOffset(), fl.xPos(), fl.yPos(), fl.invertDrive(), false, false),
                    factory.createModuleConstants(
                        fr.steerMotorId(), fr.driveMotorId(), fr.encoderId(),
                        fr.encoderOffset(), fr.xPos(), fr.yPos(), fr.invertDrive(), false, false),
                    factory.createModuleConstants(
                        bl.steerMotorId(), bl.driveMotorId(), bl.encoderId(),
                        bl.encoderOffset(), bl.xPos(), bl.yPos(), bl.invertDrive(), false, false),
                    factory.createModuleConstants(
                        br.steerMotorId(), br.driveMotorId(), br.encoderId(),
                        br.encoderOffset(), br.xPos(), br.yPos(), br.invertDrive(), false, false),
                };
            }

            /** Shared module constants factory for the competition chassis. */
            private static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> compChassisFactory() {
                return new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                    .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                    .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                    .withDriveMotorGearRatio(6.026785714285714)
                    .withSteerMotorGearRatio(26.09090909090909)
                    .withCouplingGearRatio(3.857142857142857)
                    .withWheelRadius(Inches.of(2.0))
                    .withSpeedAt12Volts(MetersPerSecond.of(5.12))
                    .withSlipCurrent(Amps.of(120))
                    .withSteerMotorGains(new Slot0Configs()
                        .withKP(110).withKI(0).withKD(5.0)
                        .withKS(0.1).withKV(2.49).withKA(0)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                    .withDriveMotorGains(new Slot0Configs()
                        .withKP(7).withKI(0).withKD(0)
                        .withKS(1).withKV(0.12))
                    .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                    .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
                    .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                    .withSteerInertia(KilogramSquareMeters.of(0.01))
                    .withDriveInertia(KilogramSquareMeters.of(0.01))
                    .withSteerFrictionVoltage(Volts.of(0.2))
                    .withDriveFrictionVoltage(Volts.of(0.2))
                    .withSteerMotorInitialConfigs(new TalonFXConfiguration()
                        .withCurrentLimits(new CurrentLimitsConfigs()
                            .withStatorCurrentLimit(Amps.of(60))
                            .withStatorCurrentLimitEnable(true)
                        )
                    );
            }
        }

        //#region Autos

        public static final class AutoConstants {
            
            public static final PIDConstants translationPID = new PIDConstants(9,0,0.01);
            public static final PIDConstants rotationPID = new PIDConstants(6.0,0,0.01);
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
            public static final double defaultShootWaitTime = 3.0;
            public static final double bumpDriveContinueTime = 0.2;

            public static final Time endTriggerDebounce = Seconds.of(0.04);
            public static final Rotation2d rotationTolerance = Rotation2d.fromDegrees(3.0);
            public static final Distance positionTolerance = Centimeter.of(1.5);
            public static final LinearVelocity speedTolerance = InchesPerSecond.of(2);

            public static final Translation2d towerPose = new Translation2d(1.061, 3.745);
            public static final Translation2d centerPose = new Translation2d(8.271, 4.035);
            public static final Translation2d hubPose = new Translation2d(4.625, 4.035);
            public static final Translation2d outpostPose = new Translation2d(0.0, 0.666);
            public static final Translation2d depotPose = new Translation2d(0.0, 5.964);

            public static final Translation2d towerTransform = new Translation2d(0.0, 0.49075);
            public static final Translation2d trenchTransform = new Translation2d(0, -3.4);
            public static final Translation2d bumpTransform = new Translation2d(0, -1.523);
            public static final Translation2d bumpTrenchDivTransform = new Translation2d(0, 2.604);
            public static final Translation2d approachTransform = new Translation2d(-1.1, 0);
            public static final Translation2d trenchExitTransform = new Translation2d(1.5, 0);
            public static final Translation2d bumpExitTransform = new Translation2d((centerPose.getX() - hubPose.getX()) * 1.2, 0);
            public static final Translation2d fuelIntakeTransform = new Translation2d(0, 1.5);

            public static final Distance robotLength = Millimeters.of(818.5);
            public static final Distance robotWidth = Millimeters.of(875.65);
            public static final Distance intakeLength = Millimeters.of(213.05);
            public static final Distance towerPadding = Inches.of(10);
            public static final Distance outpostPadding = Inches.of(6);
            public static final Distance paddingFromOp = Inches.of(3); // Padding away from center so we don't hit opponent robots
            public static final Distance paddingFromCenter = Meters.of(0.75); // Padding from center so we don't hit friendly robots (and implement u-turn movement)
            public static final Distance bumperThickness = Millimeters.of(72.7);

            public static final PathConstraints defaultPathConstraints = new PathConstraints(
                2.0,
                3.0,
                1.0 * Math.PI,
                2.0 * Math.PI
            );

            public static final PathConstraints trenchPathConstraints = new PathConstraints(
                2.0,
                2.0,
                1.0 / 2.0 * Math.PI,
                1.0 * Math.PI
            );

            public static final PathConstraints bumpPathConstraints = new PathConstraints(
                2.0,
                2.0,
                1.0 * Math.PI,
                2.0 * Math.PI
            );

            public static final PathConstraints intakePathConstraints = new PathConstraints(
                2.0,
                5.0,
                2.0 * Math.PI,
                1.0 * Math.PI
            );

            public static final PathConstraints climbPathConstraints = new PathConstraints(
                1.5,
                2.0,
                1.0 * Math.PI,
                1.0 * Math.PI
            );

            public static final Rotation2d trenchApproachAngle = Rotation2d.fromDegrees(0.0);
            public static final Rotation2d bumpApproachAngle = Rotation2d.fromDegrees(45.0);

            public enum PathplannerConfigs {
                TEST_CHASSIS(new RobotConfig(
                    Pounds.of(15),
                    KilogramSquareMeters.of(3),
                    new com.pathplanner.lib.config.ModuleConfig(
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
                    Pounds.of(160),
                    KilogramSquareMeters.of(2),
                    new com.pathplanner.lib.config.ModuleConfig(
                        Inches.of(2.0),
                        MetersPerSecond.of(5.12),
                        2.255,
                        DCMotor.getKrakenX60Foc(1),
                        6.026785714285714,
                        Amps.of(120),
                        1
                    ),
                    new Translation2d(Inches.of(9.585892).in(Meter),  Inches.of(12.1640885).in(Meter)),  // Front left
                    new Translation2d(Inches.of(9.585892).in(Meter),  Inches.of(-12.1640885).in(Meter)), // Front right
                    new Translation2d(Inches.of(-9.585892).in(Meter), Inches.of(12.1640885).in(Meter)),  // Back left
                    new Translation2d(Inches.of(-9.585892).in(Meter), Inches.of(-12.1640885).in(Meter))  // Back right
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
        public static final AprilTagFieldLayout apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        
        public static final SimCameraProperties simCameraProperties = new SimCameraProperties();
            static {
                simCameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(97.65));
                simCameraProperties.setCalibError(0.05, 0.005);
                simCameraProperties.setFPS(50);
                simCameraProperties.setAvgLatencyMs(20);
                simCameraProperties.setLatencyStdDevMs(0);
            }

        public static final class StdDevConstants {
            public static final double baseXYStdDev = 0.32;  // 0.18
            public static final double baseThetaStdDev = 0.87;  // 0.5 — heading from vision is still less reliable
            public static final double ambiguityWeight = 1.0;  // 0.6
            public static final double areaWeight = 0.9;  // 0.6
            public static final double motionWeight = 0.3;  // 0.4
            public static final double latencyWeight = 1.0;  // 0.4
        }

        public static final class CameraConstants {

            /** Currently the transform of the evan camera */
            public static final Transform3d frontTowerCamTransform = new Transform3d(
                new Translation3d(
                    -0.119170, 
                    0.310900,
                    0.520276
                ),
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-30), 
                    Math.toRadians(0)
                )
            );

            /** Currently the transform of the daniil camera */
            public static final Transform3d rioCamTransform =  new Transform3d(
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

            /** Currently the transform of the val camera */
            public static final Transform3d backTowerCamTransform =  new Transform3d(
                new Translation3d(
                    -0.266691, 
                    0.311499, 
                    0.437110 + 0.0127
                ),
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-30), 
                    Math.toRadians(180)
                )
            );

            /** Currently the transform of the gollum camera */
            public static final Transform3d swerveCamTransform =  new Transform3d(
                new Translation3d(
                    -0.2868215, 
                    -0.1771345, 
                    0.2412915
                ),
                new Rotation3d(
                    Math.toRadians(0), 
                    Math.toRadians(-23), 
                    Math.toRadians(180)
                )
            );

            //-----------------------------------------------------

            public static final Map<String, ProcessorInterface> cameras = Map.of(
                "evan", new PhotonProcessor(
                    "evan", 
                    apriltagFieldLayout, 
                    frontTowerCamTransform, 
                    new StdDevCalculator(1.3), 
                    simCameraProperties
                ),
                "daniil", new PhotonProcessor(
                    "daniil", 
                    apriltagFieldLayout, 
                    rioCamTransform, 
                    new StdDevCalculator(1.3), 
                    simCameraProperties
                ),
                "val", new PhotonProcessor(
                    "val", 
                    apriltagFieldLayout, 
                    backTowerCamTransform, 
                    new StdDevCalculator(1.3), 
                    simCameraProperties
                )
            );
        }
    }

    //#endregion
    //#region Superstructure

    public static final class SuperstructureConstants {
        
        public static final Translation2d turretTranslation2D = new Translation2d(-0.1185, -0.1568);
        public static final Translation3d turretTranslation3D = new Translation3d(turretTranslation2D.getX(), turretTranslation2D.getY(), Units.inchesToMeters(21.443748 + 2.955));

        public static final double shooterReadyThresholdRPS = 5.0;
        public static final Rotation2d pivotSafeThreshold = Rotation2d.fromDegrees(100);
        public static final Rotation2d turretMinSafeThreshold = Rotation2d.fromDegrees(-10);
        public static final Rotation2d turretMaxSafeThreshold = Rotation2d.fromDegrees(10);

        public static final Distance bumpLength = Inches.of(48.93);
        public static final Distance trenchLength = Inches.of(50);

        public static final double towerXTransform = 0.5305;
        public static final double towerYTransform = 0.49075;

        /**
         * How far ahead (in seconds) to project the robot's position when determining
         * whether to lower the hood for the trench. At higher speeds the projected
         * position will reach the trench boundary sooner, triggering the hood retract
         * earlier so it isn't clipped.
         */
        public static final double TRENCH_LOOKAHEAD_SEC = 0.5;

        public static final double PIPELINE_RATE_LIMIT_SEC = 0.2;
        public static final double PIVOT_JOSTLE_FREQUENCY = 2.0; // Hz

        public static final double percentLoss = 0.05; // Percent loss on shooter to ball transfer

    }

    //#endregion
    //#region Auto-Aim

    public static final class AutoAimConstants {
        /** Throttle rate for simulation projectile spawning (seconds between shots). */
        public static final double SIM_SHOT_INTERVAL_SECONDS = 0.1;
        
        /**
         * Height of the hub target used for both aim calculation and sim display.
         * The aim calculator targets the top of the hub; the sim tolerance check
         * uses a slightly lower center point for a more forgiving hit window.
         */
        public static final double HUB_HEIGHT = Units.inchesToMeters(72);
        public static final double FUNNEL_HEIGHT = Units.inchesToMeters(22.25);
        public static final Translation3d HUB_POSITION = new Translation3d(hubPose.getX(), hubPose.getY(), HUB_HEIGHT);
        public static final Translation3d BOTTOM_FUNNEL_POSITION = new Translation3d(hubPose.getX(), hubPose.getY(), HUB_HEIGHT - FUNNEL_HEIGHT);
        public static final double MAX_SHOOTER_RPS = 100;

        public static final Distance HUB_SHOT_PADDING = Meters.of(0.1);
        public static final Distance HUB_IDEAL_SHOT_PADDING = Meters.of(0.5);

        public static final double alliancePassOffset = 3.14;

        public static final Translation2d leftPassTarget2d = new Translation2d(2, hubPose.getY() + alliancePassOffset);
        public static final Translation2d rightPassTarget2d = new Translation2d(2, hubPose.getY() - alliancePassOffset);

        public static final Translation3d leftPassTarget = new Translation3d(leftPassTarget2d.getX(), leftPassTarget2d.getY(), 0);
        public static final Translation3d rightPassTarget = new Translation3d(rightPassTarget2d.getX(), rightPassTarget2d.getY(), 0);
    }

    //#endregion
    //#region Shooter

    public static class ShooterConstants {
        
        public static final int LEAD_MOTOR_ID = 22;
        public static final int FOLLOWER_MOTOR_ID = 23;

        /** Idle revolutions-per-second to hold when robot is enabled but not actively shooting. */
        public static final double IDLE_SHOOTER_RPS = 20.0;

        public static final double P = 0.4;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double V = 0.123;
        public static final double A = 30;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 80;
        public static final double LOWER_LIMIT = 60;

        public static final double LOWER_TIME = 6;
        public static final double MOTOR_MECHANISM_RATIO = 0.9375;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
            .withKV(V)
            .withKA(A);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(LOWER_LIMIT)
            .withSupplyCurrentLowerTime(LOWER_TIME);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(MOTOR_MECHANISM_RATIO);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast);

        public static final boolean ENABLE_FOC = false;
    }

    //#endregion
    //#region Hood
    
    public static class HoodConstants {

        public static final int MOTOR_ID = 21;

        public static final double P = 280.0;
        public static final double I = 0.0;
        public static final double D = 0.2;
        public static final double V = 0.5;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 85.3333333;

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

        public static final boolean ENABLE_FOC = true;
    }

    //#endregion
    //#region Pivot

    public static class PivotConstants {

        public static final int MOTOR_ID = 14;
        public static final int ENCODER_ID = 16;

        public static final double P = 120;
        public static final double I = 0.0;
        public static final double D = 0.2;

        public static final double MAX_VELOCITY = 8;
        public static final double MAX_ACCELERATION = 8;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 50.625;

        public static final double MAGNET_OFFSET = -0.130371;
        public static final SensorDirectionValue ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-2);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(130);

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
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(ENCODER_ID)
            .withRotorToSensorRatio(MOTOR_MECHANISM_RATIO)
            .withSensorToMechanismRatio(1.0);

        public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        public static final boolean ENABLE_FOC = true;
    }

    //#endregion
    //#region Feeder

    public static class FeederConstants {
        
        public static final int MOTOR_ID = 18;

        public static final double P = 0.3;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double V = 0.24;

        public static final double MAX_RPS = 100;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 2;
        public static final double MOTOR_MECHANISM_RATIO = 20.0 / 9.0;

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
            .withNeutralMode(NeutralModeValue.Coast);

        public static final boolean ENABLE_FOC = false;
    }

    //#endregion
    //#region Indexer

    public static class IndexerConstants {
        
        public static final int MOTOR_ID = 17;

        public static final double P = 0.5;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double V = 0.462;

        public static final double MAX_RPS = 100;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

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
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);
        
        public static final boolean ENABLE_FOC = false;
    }

    //#endregion
    //#region Turret

    public static class TurretConstants{

        public static final int MOTOR_ID = 19;
        public static final int ENCODER_ID = 20;

        public static final double P = 300;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double V = 1;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

        public static final double LOWER_TIME = 1;
        public static final double MOTOR_MECHANISM_RATIO = 1.0 / ((12.0/38.0) * (18.0/38.0) * (11.0/84.0));
        public static final double ENCODER_MECHANISM_RATIO = 11.0 / 84.0;
        public static final double MAGNET_OFFSET = 0.424805;

        public static final SensorDirectionValue ENCODER_SENSOR_DIRECTION = SensorDirectionValue.Clockwise_Positive;

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
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        
        public static final boolean ENABLE_FOC = true;
    }

    //#endregion
    //#region Intake
    
    public static class IntakeConstants {

        public static final int MOTOR_ID = 15;

        public static final double MAX_RPS = 100;

        public static final double P = 0.08;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double V = 0.37;

        public static final boolean CURRENT_LIMIT_ENABLE = true;
        public static final double CURRENT_LIMIT = 40;
        public static final double LOWER_LIMIT = 20;

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
            .withNeutralMode(NeutralModeValue.Coast);

        public static final boolean ENABLE_FOC = false;
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

        public static final double MIN_HEIGHT = -3.5;
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

        public static final boolean ENABLE_FOC = true;
    }

    //#endregion

}
