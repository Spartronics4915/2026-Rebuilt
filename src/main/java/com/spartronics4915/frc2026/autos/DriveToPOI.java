package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.spartronics4915.frc2026.commands.PositionPIDCommand;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveToPOI {
    private final SwerveSubsystem swerve;
    
    public DriveToPOI(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public enum POI {
        TOWER,
        DEPOT,
        OUTPOST;
    }

    public Command generateCommand(POI poi) {
        return Commands.defer(() -> {
            if (swerve != null && swerve.getRelativePose().getX() > hubPose.getX()) {
                return Commands.runOnce(() -> {
                    System.out.println("Past hub, cannot drive to POI");
                });
            }

            switch (poi) {
                case TOWER: {
                    boolean shouldFlip = swerve.getRelativePose().getY() < towerPose.getY();
                    return Commands.sequence(
                        Commands.parallel(
                            // Move climber up while driving up,
                            Autos.generatePathFromWaypoint(
                                swerve,
                                towerPose.plus(
                                    towerTransform.plus(
                                        new Translation2d(0, towerPadding.in(Meters))
                                    ).plus(
                                        new Translation2d(0, robotLength.in(Meters) / 2.0)
                                    ).times(shouldFlip ? -1 : 1)
                                ), 
                                shouldFlip ? Rotation2d.fromDegrees(270.0) : Rotation2d.fromDegrees(90.0),
                                shouldFlip ? Rotation2d.fromDegrees(90.0) : Rotation2d.fromDegrees(270.0)
                            )
                        ).finallyDo(
                            (interrupted) -> {
                                // Put climber back down only if interrupted since the command got canceled on the way there
                            }
                        ),
                        PositionPIDCommand.generateCommand(
                            swerve,
                            Autos.flipIfNeeded(
                                swerve,
                                new Pose2d(
                                    towerPose.plus(
                                        towerTransform.plus(
                                            new Translation2d(0, robotLength.in(Meters) / 2.0)
                                        ).times(shouldFlip ? -1 : 1)
                                    ),
                                    shouldFlip ? Rotation2d.fromDegrees(270.0) : Rotation2d.fromDegrees(90.0)
                                )
                            ),
                            Seconds.of(2.0)
                        ),
                        Commands.runOnce(() -> {
                            // CommandScheduler.getInstance().schedule(); // Pull climber back down to move robot up
                        })
                    );
                }
                case DEPOT: {
                    return Autos.generatePathFromWaypoint(
                        swerve,
                        depotPose.plus(
                            new Translation2d(robotLength.in(Meters) / 2.0 + intakeLength.in(Meters), 0)
                        ),
                        Rotation2d.fromDegrees(180.0),
                        Rotation2d.fromDegrees(180.0)
                    );
                }
                case OUTPOST: {
                    return Autos.generatePathFromWaypoint(
                        swerve,
                        outpostPose.plus(
                            new Translation2d(robotWidth.in(Meters) / 2.0, 0)
                        ),
                        Rotation2d.fromDegrees(90.0),
                        Rotation2d.fromDegrees(180.0)
                    );
                }
                default: {
                    return Commands.runOnce(() -> {
                        System.out.println("Invalid POI");
                    });
                }
            }
        }, Set.of(swerve));
    }
}
