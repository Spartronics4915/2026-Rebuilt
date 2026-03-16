package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NeutralZoneAutos {
    private final SwerveSubsystem swerve;

    public NeutralZoneAutos(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    public Command generateQuadrantCommand(boolean isRightSide, boolean endWithSpeed) {
        return Commands.defer(() -> {
            double sideMultiplier = isRightSide ? -1 : 1;
            Rotation2d rotation = Rotation2d.fromDegrees(isRightSide ? 90 : -90);

            Translation2d offsetFromCenter = new Translation2d(
                -robotWidth.in(Meters) / 2 - centerPadding.in(Meters),
                (robotLength.in(Meters) / 2 + intakeLength.in(Meters)) * sideMultiplier
            );

            Pose2d intakeStart = new Pose2d(
                centerPose.plus(offsetFromCenter).plus(fuelIntakeTransform.times(sideMultiplier)),
                rotation
            );
            Pose2d quadrantEnd = new Pose2d(centerPose.plus(offsetFromCenter), rotation);

            List<Pose2d> poses = new ArrayList<>(List.of(intakeStart, quadrantEnd));

            Autos.addStartingPoseToPath(swerve, poses);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                List.of(new RotationTarget(1, rotation)),
                List.of(),
                List.of(new ConstraintsZone(1, 2, intakePathConstraints)),
                List.of(),
                defaultPathConstraints,
                Autos.generateStartingState(swerve),
                new GoalEndState(endWithSpeed ? intakePathConstraints.maxVelocity() : MetersPerSecond.of(0), rotation),
                false
            );
            
            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }

    public Command generateHalfCommand(boolean isRightSide, boolean endWithSpeed) {
        return Commands.defer(() -> {
            double sideMultiplier = isRightSide ? -1 : 1;
            Rotation2d rotation = Rotation2d.fromDegrees(isRightSide ? 90 : -90);

            Translation2d startOffset = new Translation2d(
                -robotWidth.in(Meters) / 2 - centerPadding.in(Meters),
                (robotLength.in(Meters) / 2 + intakeLength.in(Meters)) * sideMultiplier
            ).plus(fuelIntakeTransform.times(sideMultiplier));

            Translation2d endOffset = new Translation2d(
                -robotWidth.in(Meters) / 2 - centerPadding.in(Meters),
                0
            ).plus(fuelIntakeTransform.times(-sideMultiplier));

            Pose2d fuelStart = new Pose2d(centerPose.plus(startOffset), rotation);
            Pose2d fuelEnd = new Pose2d(centerPose.plus(endOffset), rotation);

            List<Pose2d> poses = new ArrayList<>(List.of(fuelStart, fuelEnd));

            Autos.addStartingPoseToPath(swerve, poses);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                List.of(new RotationTarget(1, rotation)),
                List.of(),
                List.of(new ConstraintsZone(1, 2, intakePathConstraints)),
                List.of(),
                defaultPathConstraints,
                Autos.generateStartingState(swerve),
                new GoalEndState(endWithSpeed ? intakePathConstraints.maxVelocity() : MetersPerSecond.of(0), rotation),
                false
            );

            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }   
}
