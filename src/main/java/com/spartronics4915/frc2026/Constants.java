// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import java.util.Arrays;
import java.util.Optional;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class SwerveConstants {
       public enum SwerveDirectories{
        COMP_CHASSIS("swerve/comp-chassis");

        public String directory;

        private SwerveDirectories(String directory) {
            this.directory = directory;
        }
       }

       public static final String posePublisher = null;
       public static final LinearVelocityUnit MetersPerSecond = null;
       public static final double kTrackWidth = Units.inchesToMeters(0);
       public static final double kWheelbase = Units.inchesToMeters(0);
       public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);
       
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(0);
        public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(kMaxSpeed.in(MetersPerSecond) * Math.PI / kChassisRadius);

        public static final class AutoConstants {
            public static final PIDConstants kTranslationPID = new PIDConstants(0, 0, 0);
            public static final PIDConstants kRotationPID = new PIDConstants(0, 0, 0);
            public static final String PathPlannerConfigs = null;

            public enum PathplannerConfigs{
                COMP_CHASSIS(new RobotConfig(
                    Kilogram.of(0),
                    KilogramSquareMeters.of(0),
                    new ModuleConfig(
                        Inches.of(0),
                        MetersPerSecond.of(4),
                        0,
                        DCMotor.getNEO(0),
                        0,
                        Amps.of(0),
                        0
                    ),
                    new Translation2d(Inches.of(0), Inches.of(0)),
                    new Translation2d(Inches.of(0), Inches.of(0)),
                    new Translation2d(Inches.of(0), Inches.of(0)),
                    new Translation2d(Inches.of(0), Inches.of(0))
                ));

                public RobotConfig config;

                private PathplannerConfigs(RobotConfig config) {
                    this.config = config;
                }
            }
        }
    }

    public static class OdometryConstants {
        //public static final double kMaxSwerveVisionPoseDifference = 0;
    }
}
