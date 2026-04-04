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
        return Commands.defer(() -> {
            if (method.isTrench) {
                return generateTrenchCommand(method.isRightSide, toNeutralZone, endWithSpeed);
            } else {
                return generateBumpCommand(method.isRightSide, toNeutralZone, endWithSpeed);
            }
        }, Set.of(swerve));
    }

    public Command generateBumpCommand(boolean isRightSide, boolean toNeutralZone, boolean endWithSpeed) {
        Rotation2d LRFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg; // Left/Right flip
        Rotation2d IOFlip = toNeutralZone ? Rotation2d.kZero : Rotation2d.k180deg; // In/Out flip

        List<PathElement> pathElements = new ArrayList<PathElement>(List.of(
            new Path.Waypoint(
                hubPose.plus(
                    bumpTransform.rotateBy(LRFlip)
                ).plus(
                    approachTransform.rotateBy(IOFlip)
                ),
                bumpApproachAngle.times((isRightSide == toNeutralZone) ? 1 : -1).rotateBy(IOFlip)
            ),
            new Path.Waypoint(
                hubPose.plus( // Pose will be really wrong over the bump so set the setpoint *way* farther
                    exitTransform.rotateBy(IOFlip)
                ).plus(
                    bumpTransform.rotateBy(LRFlip)
                ),
                bumpApproachAngle.times((isRightSide == toNeutralZone) ? 1 : -1).rotateBy(IOFlip)
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

    public Command generateTrenchCommand(boolean isRightSide, boolean toNeutralZone, boolean endWithSpeed) {
        Rotation2d LRFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg; // Left/Right flip
        Rotation2d IOFlip = toNeutralZone ? Rotation2d.kZero : Rotation2d.k180deg; // In/Out flip

        List<PathElement> pathElements = new ArrayList<PathElement>(List.of(
            new Path.Waypoint(
                hubPose.plus(
                    trenchTransform.rotateBy(LRFlip)
                ).plus(
                    approachTransform.rotateBy(IOFlip)
                ),
                trenchApproachAngle.rotateBy(LRFlip)
            ),
            new Path.Waypoint(
                hubPose.plus(
                    trenchTransform.rotateBy(LRFlip)
                ).plus(
                    toNeutralZone
                        ? approachTransform.rotateBy(IOFlip.plus(Rotation2d.k180deg))
                        : trenchExitTransform.rotateBy(IOFlip)
                ),
                trenchApproachAngle.rotateBy(LRFlip)
            )
        ));

        Autos.removePastPoses(swerve, pathElements, toNeutralZone);

        Path path = new Path(pathElements, Autos.generatePathConstraintZone(trenchPathConstraints, 1, 2));

        return Autos.build(path, endWithSpeed ? Rotation2d.kZero.rotateBy(IOFlip) : null, swerve);
    }
}
