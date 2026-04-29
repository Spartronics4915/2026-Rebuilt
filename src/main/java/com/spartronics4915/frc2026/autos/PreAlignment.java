package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;

import java.util.Map;
import java.util.Set;

import com.spartronics4915.frc2026.autos.ComplexAutoChooser.AutoSegment;
import com.spartronics4915.frc2026.autos.ZoneTransition.TraversalMethod;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PreAlignment {
    private final SwerveSubsystem swerveSubsystem;

    public PreAlignment(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public Command generateCommand(AutoSegment prevSegment, AutoSegment nextSegment) {
        return Commands.defer(
            () -> {
                TraversalMethod prevMethod = convertToTraversalMethod(prevSegment);
                TraversalMethod nextMethod = convertToTraversalMethod(nextSegment);
                
                if (prevMethod == null || nextMethod == null) {
                    return Commands.none();
                }

                if (prevMethod.isRightSide == nextMethod.isRightSide && prevMethod.isTrench == nextMethod.isTrench) {
                    return Commands.none();
                }

                double LRFlip = nextMethod.isRightSide ? 0.0 : -180.0; // Left/Right flip

                Translation2d trans = hubPose.plus(
                    (nextMethod.isTrench ? trenchTransform : bumpTransform).rotateBy(Rotation2d.fromDegrees(LRFlip))
                ).plus(
                    approachTransform
                );

                Rotation2d rotation;
                if (nextMethod.isTrench) {
                    rotation = trenchApproachAngle.rotateBy(Rotation2d.fromDegrees(LRFlip));
                } else {
                    rotation = bumpApproachAngle.times((nextMethod.isRightSide) ? 1 : -1);
                }

                return Autos.generatePathFromWaypoint(swerveSubsystem, trans, rotation, alignPathConstraints);
            }, 
            Set.of(swerveSubsystem)
        );
    }

    private static final Map<AutoSegment, TraversalMethod> segmentToTraversalMethodMap = Map.of(
        AutoSegment.L_TRENCH_TO_NEUTRAL, TraversalMethod.LEFT_TRENCH,
        AutoSegment.L_BUMP_TO_NEUTRAL, TraversalMethod.LEFT_BUMP,
        AutoSegment.R_TRENCH_TO_NEUTRAL, TraversalMethod.RIGHT_TRENCH,
        AutoSegment.R_BUMP_TO_NEUTRAL, TraversalMethod.RIGHT_BUMP,
        AutoSegment.L_TRENCH_TO_ALLIANCE, TraversalMethod.LEFT_TRENCH,
        AutoSegment.L_BUMP_TO_ALLIANCE, TraversalMethod.LEFT_BUMP,
        AutoSegment.R_TRENCH_TO_ALLIANCE, TraversalMethod.RIGHT_TRENCH,
        AutoSegment.R_BUMP_TO_ALLIANCE, TraversalMethod.RIGHT_BUMP
    );

    /**
     * Converts an auto segment to a traversal method or returns null if 
     * the segment cannot be converted
     * @param segment Auto segment to convert
     * @return Traversal method or null if conversion is not possible
     */
    public static TraversalMethod convertToTraversalMethod(AutoSegment segment) {
        return segmentToTraversalMethodMap.get(segment);
    }
}
