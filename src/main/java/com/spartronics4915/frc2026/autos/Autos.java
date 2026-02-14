// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    public static Command nothingAuto() {
        return Commands.runOnce(() -> {
            System.out.println("Nothing Auto");
        });
    }

    public static IdealStartingState generateStartingState(SwerveSubsystem swerve) {
        LinearVelocity startingVel = MetersPerSecond.of(
            Math.max(
                getVelocityMagnitude(swerve.getFieldVelocity()).in(MetersPerSecond),
                0.1
            )
        );

        return new IdealStartingState(
            startingVel,
            swerve.getRelativeHeading().rotation().toRotation2d()
        );
    }

    public static void addStartingPoseToPath(SwerveSubsystem swerve, List<Pose2d> waypoints) {
        Pose2d pose = flipIfNeeded(
            swerve,
            new Pose2d(
                swerve.getPose().getTranslation(), 
                getPathVelocityHeading(swerve, waypoints.get(0))
            )
        );

        waypoints.add(0, pose);
    }

    public static Command generatePathFromWaypoint(SwerveSubsystem swerve, Translation2d translation, Rotation2d endingHeading, Rotation2d endingVelocityHeading) {
        Pose2d waypoint = new Pose2d(translation, endingVelocityHeading);
        List<Pose2d> poses = new ArrayList<>(List.of(waypoint));

        addStartingPoseToPath(swerve, poses);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            AutoConstants.defaultPathConstraints,
            generateStartingState(swerve),
            new GoalEndState(0.0, endingHeading)
        );

        return AutoBuilder.followPath(path);
    }

    public static Rotation2d getPathVelocityHeading(SwerveSubsystem swerve, Pose2d target){
        ChassisSpeeds cs = swerve.getFieldVelocity();
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            Translation2d diff = flipIfNeeded(swerve, target).getTranslation().minus(swerve.getPose().getTranslation());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    public static LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    public static Pose2d flipIfNeeded(SwerveSubsystem swerve, Pose2d pose) {
        if (swerve.shouldFlip()) {
            return FlippingUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }
}
