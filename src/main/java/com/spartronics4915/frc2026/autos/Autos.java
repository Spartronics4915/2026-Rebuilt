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
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;

public final class Autos extends SubsystemBase {
    public static FollowPath.Builder pathBuilder;

    public static boolean surveyMode = false;
    public static Field2d surveyField = new Field2d();
    public static List<PathElement> surveyElements = new ArrayList<>();
    public static List<Pose2d> surveyPoses = new ArrayList<>();
    public static List<Pose2d> surveyPathEndPoses = new ArrayList<>();
    public static List<Integer> rawPathEndIndices = new ArrayList<>();
    public static Pose2d swervePose = new Pose2d();
    public static double surveyPathProgressMeters = 0.0;
    public static TimeVarianceAuthority timeVarianceAuthority = new TimeVarianceAuthority();

    private static Command currentSurveyCommand = null;
    
    private static Optional<Alliance> cachedAlliance = DriverStation.getAlliance();
    static {
        SmartDashboard.putData("Auto Chooser/Survey Field", surveyField);
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
        return surveyMode 
            ? Commands.none() 
            : Commands.waitSeconds(seconds);
    }

    public static void setSwervePose(Pose2d pose) {
        swervePose = pose;
        displaySurveyField();
        surveyPathProgressMeters += timeVarianceAuthority.update() * 3; // Default 1m/s, multiply for a higher value
    }

    public static void survey(Command autoCommand) {
        if (currentSurveyCommand != null) {
            currentSurveyCommand.cancel();
        }
        surveyMode = true;
        surveyPathProgressMeters = 0.0;
        surveyElements.clear();
        surveyPoses.clear();
        surveyPathEndPoses.clear();
        rawPathEndIndices.clear();

        currentSurveyCommand = Commands.sequence(
            autoCommand,
            Commands.runOnce(() -> {
                processElements();
                displaySurveyField();
                surveyMode = false;
            })
        ).ignoringDisable(true).withDeadline(Commands.waitUntil(() -> surveyMode == false)); // Make sure that if enabled while surveying the auto it cancels

        CommandScheduler.getInstance().schedule(currentSurveyCommand);
    }

    public static void processElements() {
        Rotation2d lastRotation = Rotation2d.kZero;
        
        record SurveyRotationTarget(double index, Rotation2d rotation) {}
        List<SurveyRotationTarget> rotationTargets = new ArrayList<>();

        // Works through all the path elements and either adds it to a list (Waypoint and Translation target), or adds it to a list to process later (Rotation target)
        for (int i = 0; i < surveyElements.size(); i++) {
            PathElement element = surveyElements.get(i);
            if (element instanceof Path.Waypoint waypoint) {
                surveyPoses.add(
                    new Pose2d(
                        waypoint.translationTarget().translation(),
                        waypoint.rotationTarget().rotation()
                    )
                );
                lastRotation = waypoint.rotationTarget().rotation();

            } else if (element instanceof Path.RotationTarget rotationTarget) {
                rotationTargets.add(new SurveyRotationTarget(surveyPoses.size() - 1 + rotationTarget.t_ratio(), rotationTarget.rotation()));
                lastRotation = rotationTarget.rotation();

            } else if (element instanceof Path.TranslationTarget translationTarget) {
                surveyPoses.add(
                    new Pose2d(
                        translationTarget.translation(),
                        lastRotation
                    )
                );
            }

            // End of segment has to be stored as an index until here because the angle of the next point was unknown when adding the index
            if (rawPathEndIndices.contains(i)) {
                surveyPathEndPoses.add(surveyPoses.get(Math.max(0, surveyPoses.size() - 1)));
            }
        }

        // Because rotation targets require a translation target before them, a filler swerve pose is usually added, this removes them 
        // (since when surveying, the robot doesn't move and the pose is inaccurate)
        for (int i = surveyPoses.size() - 1; i >= 0; i--) {
            Pose2d p = surveyPoses.get(i);
            Pose2d flipped = flipIfNeeded(p);
            
            if (flipped.relativeTo(swervePose).getTranslation().getNorm() <= 0.1) {
                surveyPoses.remove(i);

                for (int j = rotationTargets.size() - 1; j >= 0; j--) {
                    if (rotationTargets.get(j).index() < i) {
                        break;
                    }
                    rotationTargets.set(j, new SurveyRotationTarget(rotationTargets.get(j).index() - 1, rotationTargets.get(j).rotation()));
                }
            }
        }

        // Interpolate rotation targets (which adds a pose2d to the same list as the translation targets and waypoints) between translation previous and next rotation targets
        for (int i = rotationTargets.size() - 1; i >= 0; i--) {
            double targetIndex = rotationTargets.get(i).index();
            Rotation2d rotation = rotationTargets.get(i).rotation();
            
            int floorIndex = (int) Math.floor(targetIndex);
            int ceilIndex = (int) Math.ceil(targetIndex);

            if (floorIndex < 0) continue; // Ignore rotation targets that happen right after the first swerve pose because it can't live update

            Translation2d floorTranslation = surveyPoses.get(floorIndex).getTranslation();
            Translation2d ceilTranslation = surveyPoses.get(ceilIndex).getTranslation();

            double interpolationRatio = targetIndex - floorIndex;
            
            Translation2d interpolatedTranslation = floorTranslation.interpolate(ceilTranslation, interpolationRatio);
            
            int insertionIndex = floorIndex + 1;
            surveyPoses.add(insertionIndex, new Pose2d(interpolatedTranslation, rotation));
        }
    }

    public static void displaySurveyField() {
        List<Pose2d> finalWaypoints = new ArrayList<>();

        // Elastic expects > some amount of elements to draw a continuous trajectory line
        for (int i = 0; i < 10; i++) {
            finalWaypoints.add(swervePose);
        }

        for (Pose2d waypoint : surveyPoses) {
            finalWaypoints.add(flipIfNeeded(waypoint));
        }

        surveyField.getObject("waypoints").setPoses(finalWaypoints);

        for (int i = 0; i < surveyPathEndPoses.size(); i++) {
            surveyField.getObject("pathEnd" + i).setPose(flipIfNeeded(surveyPathEndPoses.get(i)));
        }
        // Clear out up to a reasonable bound of previous endpoints so they don't linger ghosted
        for (int i = surveyPathEndPoses.size(); i < 20; i++) {
            surveyField.getObject("pathEnd" + i).setPoses(new Pose2d[0]);
        }

        surveyField.setRobotPose(swervePose);

        // Simulate robot position along the survey path by interpolating between every waypoint.
        double surveyPathMetersLeft = surveyPathProgressMeters;
        for (int i = 1; i < finalWaypoints.size(); i++) {
            double dist = finalWaypoints.get(i).getTranslation().getDistance(finalWaypoints.get(i - 1).getTranslation());
            if (dist > surveyPathMetersLeft) {
                Pose2d prevPoint = finalWaypoints.get(i - 1);
                Pose2d nextPoint = finalWaypoints.get(i);

                Translation2d simRobotTranslation = prevPoint.getTranslation().interpolate(nextPoint.getTranslation(), surveyPathMetersLeft / dist);
                Rotation2d simRobotRotation = prevPoint.getRotation().interpolate(nextPoint.getRotation(), surveyPathMetersLeft / dist);
                
                Pose2d simRobotPose = new Pose2d(simRobotTranslation, simRobotRotation);
                surveyField.getObject("RobotSim").setPose(simRobotPose);
                break;
            }

            surveyPathMetersLeft -= dist;
            // If it's at the end of the path, the index will be max and the break above wouldn't have triggered, triggering this
            if (i == finalWaypoints.size() - 1) surveyPathProgressMeters = 0;
        }
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
        if (surveyMode) {
            surveyElements.addAll(path.getPathElements());
            rawPathEndIndices.add(surveyElements.size() - 1);
            return Commands.none();
        }
        
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
        if (surveyMode || !toNeutralZone) {
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
