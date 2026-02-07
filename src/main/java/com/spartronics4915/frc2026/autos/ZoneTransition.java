package com.spartronics4915.frc2026.autos;

import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
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
        }, Set.of());
    }

    public Command generateCommand(TraversalMethod method, boolean toNeutralZone) {
        return Commands.defer(() -> {
            if (method.isTrench) {
                return generateTrenchCommand(method.isRightSide, toNeutralZone);
            } else {
                return generateBumpCommand(method.isRightSide, toNeutralZone);
            }
        }, Set.of());
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
                    exitTransform.rotateBy(Rotation2d.fromDegrees(IOFlip))
                ).plus(
                    bumpTransform.rotateBy(Rotation2d.fromDegrees(LRFlip))
                ),
                Rotation2d.fromDegrees(IOFlip)
            )
        ));

        poses.add(
            0, 
            flipIfNeeded(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    getPathVelocityHeading(swerve.getFieldVelocity(), poses.get(0))
                )
            )
        );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        LinearVelocity startingVel = MetersPerSecond.of(
            Math.max(
                getVelocityMagnitude(swerve.getFieldVelocity()).in(MetersPerSecond),
                0.1
            )
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            List.of(new RotationTarget(1, bumpApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip)))),
            List.of(),
            List.of(new ConstraintsZone(1, 2, bumpPathConstraints)),
            List.of(),
            defaultPathConstraints,
            new IdealStartingState(
                startingVel,
                swerve.getRelativeHeading().rotation().toRotation2d()
            ),
            new GoalEndState(0.0, bumpApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip))),
            false
        );

        return Commands.race(
            AutoBuilder.followPath(path),
            Commands.waitUntil(() -> {
                return swerve.getRelativePose().getMeasureX().in(Meters) > hubPose.getX() ^ !toNeutralZone 
                    && swerve.isFlatDebounced()
                    && vision.hasValidPose();
            })
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
                    approachTransform.rotateBy(Rotation2d.fromDegrees(IOFlip + 180.0))
                ),
                Rotation2d.fromDegrees(IOFlip)
            )
        ));

        poses.add(
            0, 
            flipIfNeeded(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    getPathVelocityHeading(swerve.getFieldVelocity(), poses.get(0))
                )
            )
        );

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        LinearVelocity startingVel = MetersPerSecond.of(
            Math.max(
                getVelocityMagnitude(swerve.getFieldVelocity()).in(MetersPerSecond),
                0.1
            )
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            List.of(new RotationTarget(1, trenchApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip)))),
            List.of(),
            List.of(new ConstraintsZone(1, 2, trenchPathConstraints)),
            List.of(),
            defaultPathConstraints,
            new IdealStartingState(
                startingVel,
                swerve.getRelativeHeading().rotation().toRotation2d()
            ),
            new GoalEndState(0.0, trenchApproachAngle.rotateBy(Rotation2d.fromDegrees(IOFlip))),
            false
        );

        return AutoBuilder.followPath(path);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            Translation2d diff = flipIfNeeded(target).getTranslation().minus(swerve.getPose().getTranslation());
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private Pose2d flipIfNeeded(Pose2d pose) {
        if (swerve.shouldFlip()) {
            return FlippingUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }
}
