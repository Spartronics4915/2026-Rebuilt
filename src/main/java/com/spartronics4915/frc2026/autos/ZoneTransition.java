package com.spartronics4915.frc2026.autos;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ZoneTransition {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    public ZoneTransition(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        this.swerve = swerveSubsystem;
        this.vision = visionSubsystem;
    }

    public enum TraversalMethod {
        LEFT_TRENCH(true, false),
        RIGHT_TRENCH(true, true),
        LEFT_BUMP(false, false),
        RIGHT_BUMP(false, true);

        public final boolean isTrench;
        public final boolean isRightSide;

        private TraversalMethod(boolean isTrench, boolean isRightSide) {
            this.isTrench = isTrench;
            this.isRightSide = isRightSide;
        }
    }

    public Command generateCommand(TraversalMethod method) {
        return Commands.defer(() -> {
            return generateCommand(method, swerve != null && swerve.getRelativePose().getX() < hubPose.getX());
        }, Set.of(swerve));
    }

    public Command generateCommand(TraversalMethod method, boolean toNeutralZone) {
        return Commands.defer(() -> {
            if (method.isTrench) {
                return generateTrenchCommand(method.isRightSide, toNeutralZone);
            } else {
                return generateBumpCommand(method.isRightSide, toNeutralZone);
            }
        }, Set.of(swerve));
    }

    public Command generateBumpCommand(boolean isRightSide, boolean toNeutralZone) {
        double LRFlip = isRightSide ? 0.0 : -180.0; // Left/Right flip
        double IOFlip = toNeutralZone ? 0.0 : -180.0; // In/Out flip

        List<Pose2d> poses = new ArrayList<>(List.of(
            new Pose2d(
                hubPose.plus(
                    bumpTransform.rotateBy(Rotation2d.fromDegrees(LRFlip))
                ).plus(
                    approachTransform.rotateBy(Rotation2d.fromDegrees(IOFlip))
                ),
                Rotation2d.fromDegrees(IOFlip)
            ),
            new Pose2d(
                hubPose.plus( // Pose will be really wrong over the bump so set the setpoint *way* farther
                    bumpExitTransform.rotateBy(Rotation2d.fromDegrees(IOFlip))
                ).plus(
                    bumpTransform.rotateBy(Rotation2d.fromDegrees(LRFlip))
                ),
                Rotation2d.fromDegrees(IOFlip)
            )
        ));

        Autos.removePastPoses(swerve, poses, toNeutralZone);

        Autos.addStartingPoseToPath(swerve, poses);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            List.of(new RotationTarget(1, bumpApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip)))),
            List.of(),
            List.of(new ConstraintsZone(1, 2, bumpPathConstraints)),
            List.of(),
            defaultPathConstraints,
            Autos.generateStartingState(swerve),
            new GoalEndState(0.0, bumpApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip))),
            false
        );

        return Commands.race(
            AutoBuilder.followPath(path),
            Commands.sequence(
                Commands.waitUntil(() -> {
                    return swerve.getRelativePose().getMeasureX().in(Meters) > hubPose.getX() ^ !toNeutralZone 
                        && swerve.isFlatDebounced()
                        && vision.hasValidPose();
                }),
                Commands.waitSeconds(bumpDriveContinueTime)
            )
        );
    }

    public Command generateTrenchCommand(boolean isRightSide, boolean toNeutralZone) {
        double LRFlip = isRightSide ? 0.0 : -180.0; // Left/Right flip
        double IOFlip = toNeutralZone ? 0.0 : -180.0; // In/Out flip

        List<Pose2d> poses = new ArrayList<>(List.of(
            new Pose2d(
                hubPose.plus(
                    trenchTransform.rotateBy(Rotation2d.fromDegrees(LRFlip))
                ).plus(
                    approachTransform.rotateBy(Rotation2d.fromDegrees(IOFlip))
                ),
                Rotation2d.fromDegrees(IOFlip)
            ),
            new Pose2d(
                hubPose.plus(
                    trenchTransform.rotateBy(Rotation2d.fromDegrees(LRFlip))
                ).plus(
                    toNeutralZone
                        ? approachTransform.rotateBy(Rotation2d.fromDegrees(IOFlip + 180.0))
                        : trenchExitTransform.rotateBy(Rotation2d.fromDegrees(IOFlip))
                ),
                Rotation2d.fromDegrees(IOFlip)
            )
        ));

        Autos.removePastPoses(swerve, poses, toNeutralZone);

        Autos.addStartingPoseToPath(swerve, poses);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            List.of(new RotationTarget(1, trenchApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip)))),
            List.of(),
            List.of(new ConstraintsZone(1, 2, trenchPathConstraints)),
            List.of(),
            defaultPathConstraints,
            Autos.generateStartingState(swerve),
            new GoalEndState(trenchPathConstraints.maxVelocityMPS(), trenchApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip))),
            false
        );

        return AutoBuilder.followPath(path);
    }
}
