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

    public static Command wait(double seconds) {
        return Commands.waitSeconds(seconds);
    }

    public static Path.PathConstraints generatePathConstraintZone(Path.PathConstraints constraints, int start, int end) {
        Path.PathConstraints limitedConstraints = new Path.PathConstraints();

        if (constraints.getMaxVelocityMetersPerSec().isPresent()) {
            limitedConstraints = limitedConstraints.setMaxVelocityMetersPerSec(new Path.RangedConstraint(constraints.getMaxVelocityMetersPerSec().get().get(0).value(), start, end));
        }
        if (constraints.getMaxAccelerationMetersPerSec2().isPresent()) {
            limitedConstraints = limitedConstraints.setMaxAccelerationMetersPerSec2(new Path.RangedConstraint(constraints.getMaxAccelerationMetersPerSec2().get().get(0).value(), start, end));
        }
        if (constraints.getMaxVelocityDegPerSec().isPresent()) {
            limitedConstraints = limitedConstraints.setMaxVelocityDegPerSec(new Path.RangedConstraint(constraints.getMaxVelocityDegPerSec().get().get(0).value(), start, end));
        }
        if (constraints.getMaxAccelerationDegPerSec2().isPresent()) {
            limitedConstraints = limitedConstraints.setMaxAccelerationDegPerSec2(new Path.RangedConstraint(constraints.getMaxAccelerationDegPerSec2().get().get(0).value(), start, end));
        }
        
        return limitedConstraints;
    }

    public static Path.PathConstraints combineConstraints(Path.PathConstraints... constraintsList) {
        Path.PathConstraints combined = new Path.PathConstraints();
        
        java.util.List<Path.RangedConstraint> vel = new java.util.ArrayList<>();
        java.util.List<Path.RangedConstraint> acc = new java.util.ArrayList<>();
        java.util.List<Path.RangedConstraint> velDeg = new java.util.ArrayList<>();
        java.util.List<Path.RangedConstraint> accDeg = new java.util.ArrayList<>();
        
        for (Path.PathConstraints c : constraintsList) {
            if (c.getMaxVelocityMetersPerSec().isPresent()) vel.addAll(c.getMaxVelocityMetersPerSec().get());
            if (c.getMaxAccelerationMetersPerSec2().isPresent()) acc.addAll(c.getMaxAccelerationMetersPerSec2().get());
            if (c.getMaxVelocityDegPerSec().isPresent()) velDeg.addAll(c.getMaxVelocityDegPerSec().get());
            if (c.getMaxAccelerationDegPerSec2().isPresent()) accDeg.addAll(c.getMaxAccelerationDegPerSec2().get());
        }
        
        if (!vel.isEmpty()) combined = combined.setMaxVelocityMetersPerSec(vel.toArray(new Path.RangedConstraint[0]));
        if (!acc.isEmpty()) combined = combined.setMaxAccelerationMetersPerSec2(acc.toArray(new Path.RangedConstraint[0]));
        if (!velDeg.isEmpty()) combined = combined.setMaxVelocityDegPerSec(velDeg.toArray(new Path.RangedConstraint[0]));
        if (!accDeg.isEmpty()) combined = combined.setMaxAccelerationDegPerSec2(accDeg.toArray(new Path.RangedConstraint[0]));
        
        return combined;
    }

    public static Command build(Path path) {
        return build(path, null, null);
    }

    public static Command build(Path path, Rotation2d endWithSpeedDirection, SwerveSubsystem swerve) {
        Translation2d overshootTarget = null;
        
        if (endWithSpeedDirection != null) {
            List<Pair<PathElement, PathElementConstraint>> waypoints = path.getPathElementsWithConstraintsNoWaypoints();
            Translation2d finalWaypoint = null;

            for (int i = waypoints.size() - 1; i >= 0; i--) {
                Path.PathElement element = waypoints.get(i).getFirst();
    
                if (element instanceof Path.RotationTarget) {
                    continue;
                }
    
                Translation2d translation = null;
                if (element instanceof Path.TranslationTarget) {
                    translation = ((Path.TranslationTarget) element).translation();
                }

                if (translation != null) {
                    finalWaypoint = translation;
                    break;
                }
            }

            if (finalWaypoint != null) {
                overshootTarget = new Translation2d(velocityEndingDistance.in(Meters), 0).rotateBy(endWithSpeedDirection).plus(finalWaypoint);
    
                path.addPathElement(new Path.TranslationTarget(overshootTarget));
            }
        }
        
        Command pathCommand = pathBuilder.build(path);
        
        // If swerve is provided and we have an overshoot target, use race to cancel when within distance
        if (swerve != null && overshootTarget != null) {
            final Translation2d overshoot = overshootTarget;
            return Commands.race(
                pathCommand,
                Commands.waitUntil(() -> {
                    double dist = swerve.getRelativePose().getTranslation().minus(overshoot).getNorm();
                    return dist <= velocityEndingDistance.in(Meters);
                })
            );
        }
        
        return pathCommand;
    }

    public static void removePastPoses(SwerveSubsystem swerve, List<Path.PathElement> waypoints, boolean toNeutralZone) {
        if (!toNeutralZone) {
            return;
        }

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

    public static Translation2d invX(Translation2d translation) {
        return new Translation2d(-translation.getX(), translation.getY());
    }

    public static Translation2d invXCond(Translation2d translation, boolean condition) {
        return condition ? invX(translation) : translation;
    }

    public static boolean shouldFlip() {
        return cachedAlliance.isPresent() && cachedAlliance.get() == Alliance.Red;
    }

    public static void resetAlliance() {
        cachedAlliance = Optional.empty();
    }
}
