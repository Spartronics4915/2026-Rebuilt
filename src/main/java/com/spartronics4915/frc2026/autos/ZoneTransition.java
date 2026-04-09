package com.spartronics4915.frc2026.autos;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;

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

    public Command generateCommand(TraversalMethod method, boolean endWithSpeed) {
        return Commands.defer(() -> {
            return generateCommand(method, swerve != null && swerve.getRelativePose().getX() < hubPose.getX(), endWithSpeed);
        }, Set.of(swerve));
    }

    public Command generateCommand(TraversalMethod method, boolean toNeutralZone, boolean endWithSpeed) {
        return generateCommand(method, toNeutralZone, endWithSpeed, false);
    }

    public Command generateCommand(TraversalMethod method, boolean toNeutralZone, boolean endWithSpeed, boolean endInTrench) {
        return Commands.defer(() -> {
            if (method.isTrench) {
                return generateTrenchCommand(method.isRightSide, toNeutralZone, endWithSpeed, endInTrench);
            } else {
                return generateBumpCommand(method.isRightSide, toNeutralZone, endWithSpeed);
            }
        }, Set.of(swerve));
    }

    public Command generateBumpCommand(boolean isRightSide, boolean toNeutralZone, boolean endWithSpeed) {
        Rotation2d LRFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg; // Left/Right flip
        Rotation2d IOFlip = toNeutralZone ? Rotation2d.kZero : Rotation2d.k180deg; // In/Out flip

        Rotation2d bumpAngle = bumpApproachAngle.times((isRightSide == toNeutralZone) ? 1 : -1).rotateBy(IOFlip);
        
        if (!toNeutralZone) {
            bumpAngle = bumpAngle.plus(Rotation2d.k180deg);
        }

        List<PathElement> pathElements = new ArrayList<PathElement>(List.of(
            new Path.Waypoint(swerve.getRelativePose()),
            new Path.RotationTarget(
                bumpAngle, 
                0.75
            ),
            new Path.Waypoint(
                new Pose2d(
                    hubPose.plus(
                        bumpTransform.rotateBy(LRFlip)
                    ).plus(
                        approachTransform.rotateBy(IOFlip)
                    ),
                    bumpAngle
                ),
                0.9
            ),
            new Path.Waypoint(
                hubPose.plus( // Pose will be really wrong over the bump so set the setpoint *way* farther
                    exitTransform.rotateBy(IOFlip)
                ).plus(
                    bumpTransform.rotateBy(LRFlip)
                ),
                bumpAngle
            )
        ));

        Autos.removePastPoses(swerve, pathElements, toNeutralZone);

        Path path = new Path(pathElements, Autos.generatePathConstraintZone(bumpPathConstraints, 1, 2));

        return Commands.race(
            Autos.build(path, endWithSpeed ? Rotation2d.kZero.rotateBy(IOFlip) : null, swerve),
            Commands.sequence(
                Commands.waitUntil(() -> {
                    return swerve.getRelativePose().getMeasureX().in(Meters) > hubPose.getX() ^ !toNeutralZone 
                        && swerve.isFlatDebounced()
                        && vision.hasAnyPose();
                }),
                Commands.waitSeconds(bumpDriveContinueTime)
            )
        );
    }

    public Command generateTrenchCommand(boolean isRightSide, boolean toNeutralZone, boolean endWithSpeed, boolean endInTrench) {
        Rotation2d LRFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg; // Left/Right flip
        Rotation2d IOFlip = toNeutralZone ? Rotation2d.kZero : Rotation2d.k180deg; // In/Out flip

        Rotation2d trenchAngle = trenchApproachAngle;

        List<PathElement> pathElements = new ArrayList<PathElement>(List.of(
            new Path.Waypoint(swerve.getRelativePose()),
            new Path.RotationTarget(
                trenchAngle, 
                0.75
            ),
            new Path.Waypoint(
                new Pose2d(
                    hubPose.plus(
                        trenchTransform.rotateBy(LRFlip)
                    ).plus(
                        approachTransform.rotateBy(IOFlip)
                    ),
                    trenchAngle
                ),
                1.2
            ),
            new Path.Waypoint(
                new Pose2d(
                    hubPose.plus(
                        trenchTransform.rotateBy(LRFlip)
                    ).plus(
                        approachTransform.rotateBy(IOFlip).times(0.4)
                    ),
                    trenchAngle
                ),
                0.2
            ),
            new Path.Waypoint(
                hubPose.plus(
                    trenchTransform.rotateBy(LRFlip)
                ).plus(
                    endInTrench
                        ? trenchExitTransform.rotateBy(IOFlip)
                        : approachTransform.rotateBy(IOFlip.plus(Rotation2d.k180deg))
                ),
                trenchAngle
            )
        ));

        Autos.removePastPoses(swerve, pathElements, toNeutralZone);

        Path path;
        if (toNeutralZone) {
            path = new Path(pathElements, Autos.generatePathConstraintZone(driveToCenterConstraints, 1, 2));
        } else {
            path = new Path(pathElements);
        }

        return Autos.build(path, endWithSpeed ? Rotation2d.kZero.rotateBy(IOFlip) : null, swerve);
    }
    
    public Command generateStartingTrenchCommand(boolean isRightSide) {
        Rotation2d LRFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg; // Left/Right flip

        Rotation2d trenchAngle = startingTrenchApproachAngle.times(isRightSide ? 1 : -1);

        List<PathElement> pathElements = new ArrayList<PathElement>(List.of(
            new Path.Waypoint(swerve.getRelativePose()),
            new Path.RotationTarget(
                trenchAngle, 
                0.75
            ),
            new Path.Waypoint(
                new Pose2d(
                    hubPose.plus(
                        trenchTransform.rotateBy(LRFlip)
                    ).plus(
                        approachTransform
                    ),
                    trenchAngle
                ),
                0.6
            ),
            new Path.Waypoint(
                hubPose.plus(
                    trenchTransform.rotateBy(LRFlip)
                ).plus(
                    approachTransform.rotateBy(Rotation2d.k180deg)
                ),
                trenchAngle
            )
        ));

        Autos.removePastPoses(swerve, pathElements, true);

        Path path = new Path(pathElements, driveToCenterConstraints);

        return Autos.build(path, Rotation2d.kZero, swerve);
    }
}
