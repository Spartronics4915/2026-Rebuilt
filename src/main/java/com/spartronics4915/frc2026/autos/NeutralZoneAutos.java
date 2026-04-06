package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;


import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;

public class NeutralZoneAutos {
    private final SwerveSubsystem swerve;

    public NeutralZoneAutos(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    public Command generateQuadrantCommand(boolean isRightSide) {
        return Commands.defer(() -> {
            double sideMultiplier = isRightSide ? -1 : 1;
            Rotation2d rotation = Rotation2d.fromDegrees(isRightSide ? 90 : -90);

            Translation2d offsetFromCenter = new Translation2d(
                -robotWidth.in(Meters) / 2 - paddingFromOp.in(Meters),
                (robotLength.in(Meters) / 2 + intakeLength.in(Meters)) * sideMultiplier
            );

            Pose2d intakeStart = new Pose2d(
                centerPose.plus(offsetFromCenter).plus(fuelIntakeTransform.times(sideMultiplier)),
                rotation
            );
            Pose2d quadrantEnd = new Pose2d(
                centerPose.plus(offsetFromCenter).plus(
                    new Translation2d(
                        0, 
                        paddingFromCenter.in(Meters)
                    ).times(sideMultiplier)
                ), 
                rotation
            );

            List<PathElement> pathElements = new ArrayList<>(List.of(
                new Path.Waypoint(swerve.getRelativePose()),
                new Path.RotationTarget(
                    intakeStart.getRotation(), 
                    0.75
                ),
                new Path.Waypoint(intakeStart, 0.75),
                new Path.Waypoint(quadrantEnd)
            ));

            Path path = new Path(
                pathElements,
                Autos.generatePathConstraintZone(intakePathConstraints, 1, 2)
            );

            return Autos.build(path, quadrantEnd.getTranslation().minus(intakeStart.getTranslation()).getAngle(), swerve);
        }, Set.of(swerve));
    }

    public Command generateHairpinCommand(boolean isRightSide) {
        return Commands.defer(() -> {
            double sideMultiplier = isRightSide ? -1 : 1;
            Rotation2d rotation = Rotation2d.fromDegrees(isRightSide ? 90 : -90);

            Translation2d offsetFromCenter = new Translation2d(
                -robotWidth.in(Meters) / 2 - paddingFromOp.in(Meters),
                (robotLength.in(Meters) / 2 + intakeLength.in(Meters)) * sideMultiplier
            );

            Pose2d intakeStart = new Pose2d(
                centerPose.plus(offsetFromCenter).plus(fuelIntakeTransform.times(sideMultiplier)),
                rotation
            );
            Pose2d quadrantEnd = new Pose2d(
                centerPose.plus(offsetFromCenter).plus(
                    new Translation2d(
                        0, 
                        paddingFromCenter.in(Meters)
                    ).times(sideMultiplier)
                ), 
                rotation
            );

            Pose2d quadrantTurnAround = new Pose2d(
                quadrantEnd.getTranslation().plus(
                    new Translation2d(-1.4, 0)
                ),
                quadrantEnd.getRotation().rotateBy(Rotation2d.k180deg)
            );

            List<PathElement> pathElements = new ArrayList<>(List.of(
                new Path.Waypoint(swerve.getRelativePose()),
                new Path.RotationTarget(
                    intakeStart.getRotation(), 
                    0.75
                ),
                new Path.Waypoint(intakeStart, 0.75),
                new Path.Waypoint(quadrantEnd),
                new Path.RotationTarget(Rotation2d.k180deg, 0.5),
                new Path.Waypoint(quadrantTurnAround, 0.1)
            ));

            Path path = new Path(
                pathElements,
                Autos.generatePathConstraintZone(intakePathConstraints, 1, 2)
            );

            return Autos.build(path, quadrantEnd.getTranslation().minus(intakeStart.getTranslation()).getAngle(), swerve);
        }, Set.of(swerve));
    }

    public Command generateInvertedQuadrantCommand(boolean toRightSide) {
        return Commands.defer(() -> {
            double sideMultiplier = toRightSide ? 1 : -1;
            Rotation2d rotation = Rotation2d.fromDegrees(toRightSide ? -90 : 90);

            Translation2d offsetFromCenter = new Translation2d(
                -robotWidth.in(Meters) / 2 - paddingFromOp.in(Meters),
                (robotLength.in(Meters) / 2 + intakeLength.in(Meters)) * sideMultiplier
            );

            Pose2d quadrantEnd = new Pose2d(
                centerPose.plus(offsetFromCenter).plus(fuelIntakeTransform.times(-sideMultiplier)),
                rotation
            );
            Pose2d intakeStart = new Pose2d(
                centerPose.plus(offsetFromCenter), 
                rotation
            );

            List<PathElement> pathElements = new ArrayList<>(List.of(
                new Path.Waypoint(swerve.getRelativePose()),
                new Path.RotationTarget(
                    intakeStart.getRotation(), 
                    0.75
                ),
                new Path.Waypoint(intakeStart, 0.75),
                new Path.Waypoint(quadrantEnd)
            ));

            Path path = new Path(
                pathElements,
                Autos.generatePathConstraintZone(intakePathConstraints, 1, 2)
            );

            return Autos.build(path, quadrantEnd.getTranslation().minus(intakeStart.getTranslation()).getAngle(), swerve);
        }, Set.of(swerve));
    }

    public Command generateHalfCommand(boolean isRightSide, boolean endWithSpeed) {
        return Commands.defer(() -> {
            double sideMultiplier = isRightSide ? -1 : 1;
            Rotation2d rotation = Rotation2d.fromDegrees(isRightSide ? 90 : -90);

            Translation2d startOffset = new Translation2d(
                -robotWidth.in(Meters) / 2 - paddingFromOp.in(Meters),
                (robotLength.in(Meters) / 2 + intakeLength.in(Meters)) * sideMultiplier
            ).plus(fuelIntakeTransform.times(sideMultiplier));

            Translation2d endOffset = new Translation2d(
                -robotWidth.in(Meters) / 2 - paddingFromOp.in(Meters),
                0
            ).plus(fuelIntakeTransform.times(-sideMultiplier));

            Pose2d fuelStart = new Pose2d(centerPose.plus(startOffset), rotation);
            Pose2d fuelEnd = new Pose2d(centerPose.plus(endOffset), rotation);

            List<PathElement> pathElements = new ArrayList<>(List.of(
                new Path.Waypoint(swerve.getRelativePose()),
                new Path.RotationTarget(
                    fuelStart.getRotation(), 
                    0.75
                ),
                new Path.Waypoint(fuelStart, 0.75),
                new Path.Waypoint(fuelEnd)
            ));

            Path path = new Path(
                pathElements,
                Autos.generatePathConstraintZone(intakePathConstraints, 1, 2)
            );

            return Autos.build(path, endWithSpeed ? fuelEnd.getTranslation().minus(fuelStart.getTranslation()).getAngle() : null, swerve);
        }, Set.of(swerve));
    }
}
