// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.robotLength;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.towerPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.velocityEndingDistance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;

public final class Autos {
    public static FollowPath.Builder pathBuilder;
    
    private static Optional<Alliance> cachedAlliance = DriverStation.getAlliance();
    static {
        new Trigger(
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ).onChange(Commands.runOnce(() -> {
            cachedAlliance = DriverStation.getAlliance();
        }).ignoringDisable(true));
    }

    public static Command nothingAuto() {
        return Commands.runOnce(() -> {
            System.out.println("Nothing Auto");
        });
    }

    public static void setPathBuilder(FollowPath.Builder builder) {
        pathBuilder = builder;
    }

    public static Command build(Path path) {
        return build(path, false);
    }

    public static Path.PathConstraints generatePathConstraintZone(Path.PathConstraints constraints, int start, int end) {
        return new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(new Path.RangedConstraint(constraints.getMaxVelocityMetersPerSec().get().get(0).value(), start, end))
            .setMaxAccelerationMetersPerSec2(new Path.RangedConstraint(constraints.getMaxAccelerationMetersPerSec2().get().get(0).value(), start, end))
            .setMaxVelocityDegPerSec(new Path.RangedConstraint(constraints.getMaxVelocityDegPerSec().get().get(0).value(), start, end))
            .setMaxAccelerationDegPerSec2(new Path.RangedConstraint(constraints.getMaxAccelerationDegPerSec2().get().get(0).value(), start, end));
    }

    public static Command build(Path path, boolean endWithSpeed) {
        if (endWithSpeed) {
            List<Pair<PathElement, PathElementConstraint>> waypoints = path.getPathElementsWithConstraintsNoWaypoints();
            Translation2d finalWaypoint = null;
            Translation2d secondFinalWaypoint = null;

            for (int i = waypoints.size() - 1; i >= 0; i--) {
                Path.PathElement element = waypoints.get(i).getFirst();
    
                if (element instanceof Path.RotationTarget) {
                    continue;
                }
    
                if (element instanceof Path.TranslationTarget) {
                    Translation2d translation = ((Path.TranslationTarget) element).translation();
                    if (finalWaypoint == null) {
                        finalWaypoint = translation;
                    } else if (secondFinalWaypoint == null) {
                        secondFinalWaypoint = translation;
                        break;
                    }
                }
            }

            if (finalWaypoint != null && secondFinalWaypoint != null) {
                Rotation2d finalDirection = finalWaypoint.minus(secondFinalWaypoint).getAngle();
                Translation2d overshootTarget = new Translation2d(velocityEndingDistance.in(Meters), 0).rotateBy(finalDirection).plus(finalWaypoint);
    
                path.addPathElement(new Path.TranslationTarget(overshootTarget));
                path.setPathConstraints(
                    new Path.PathConstraints()
                        .setEndTranslationToleranceMeters(velocityEndingDistance.in(Meters))
                        .setEndRotationToleranceDeg(10)
                );
            }
        }
        return pathBuilder.build(path);
    }

    public static void removePastPoses(SwerveSubsystem swerve, List<Path.PathElement> waypoints, boolean toNeutralZone) {
        double x = swerve.getRelativePose().getX();

        for (int i = waypoints.size() - 1; i >= 0; i--) {
            Path.PathElement p = waypoints.get(i);

            if (p instanceof Path.RotationTarget) {
                continue;
            }

            if (p instanceof Path.TranslationTarget) {
                Path.TranslationTarget t = (Path.TranslationTarget) p;
                if ((t.translation().getX() > x) ^ toNeutralZone) {
                    waypoints.remove(i);
                    waypoints.add(i, new Path.TranslationTarget(flipIfNeeded(swerve.getPose()).getTranslation()));
                }
            }

            if (p instanceof Path.Waypoint) {
                Path.Waypoint w = (Path.Waypoint) p;
                if ((w.translationTarget().translation().getX() > x) ^ toNeutralZone) {
                    waypoints.remove(i);
                    waypoints.add(i, new Path.Waypoint(flipIfNeeded(swerve.getPose())));
                }
            }
        }
    }

    public static Command generatePathFromWaypoint(SwerveSubsystem swerve, Translation2d translation, Rotation2d endingHeading) {
        return generatePathFromWaypoint(swerve, translation, endingHeading, null);
    }

    public static Command generatePathFromWaypoint(SwerveSubsystem swerve, Translation2d translation, Rotation2d endingHeading, Path.PathConstraints pathConstraints) {
        Pose2d waypoint = new Pose2d(translation, endingHeading);
        List<PathElement> pathElements = new ArrayList<>(List.of(
            new Path.Waypoint(waypoint)
        ));

        Path path;
        if (pathConstraints != null) {
            path = new Path(
                pathElements,
                pathConstraints
            );
        } else {
            path = new Path(
                pathElements
            );
        }

        return build(path);
    }

    public static Pose2d flipIfNeeded(Pose2d pose) {
        if (Autos.shouldFlip()) {
            return FlippingUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }

    public static boolean shouldFlip() {
        return cachedAlliance.isPresent() && cachedAlliance.get() == Alliance.Red;
    }

    public static void resetAlliance() {
        cachedAlliance = Optional.empty();
    }
}
