package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.EnumSet;
import java.util.Set;

import com.spartronics4915.frc2026.commands.PositionPIDCommand;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class DriveToPOI {
    private final SwerveSubsystem swerve;
    private double outpostWaitTime;
    
    public DriveToPOI(SwerveSubsystem swerve) {
        this.swerve = swerve;

        outpostWaitTime = SmartDashboard.getNumber("Auto Chooser/Outpost Wait Time", defaultOutpostWaitTime);

        SmartDashboard.putNumber("Auto Chooser/Outpost Wait Time", outpostWaitTime);

        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto Chooser");
        table.addListener("Outpost Wait Time", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (filler, key, event) -> {
            outpostWaitTime = event.valueData.value.getDouble();
        });
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

                    Translation2d climbPose = towerPose.plus(
                        towerTransform.plus(
                            new Translation2d(0, robotLength.in(Meters) / 2.0)
                        ).times(shouldFlip ? -1 : 1)
                    );
                    Translation2d climbApproachPose = climbPose.plus(
                        new Translation2d(0, towerPadding.in(Meters)).times(shouldFlip ? -1 : 1)
                    );

                    return Commands.sequence(
                        Commands.parallel(
                            // Move climber up while driving up,
                            Autos.generatePathFromWaypoint(
                                swerve,
                                climbApproachPose, 
                                shouldFlip ? Rotation2d.fromDegrees(270.0) : Rotation2d.fromDegrees(90.0),
                                shouldFlip ? Rotation2d.fromDegrees(90.0) : Rotation2d.fromDegrees(270.0)
                            )
                        ),
                        PositionPIDCommand.generateCommand(
                            swerve,
                            Autos.flipIfNeeded(
                                swerve,
                                new Pose2d(
                                    climbApproachPose,
                                    shouldFlip ? Rotation2d.fromDegrees(270.0) : Rotation2d.fromDegrees(90.0)
                                )
                            ),
                            Seconds.of(2.0)
                        ),
                        // This makes all climb operations beyond uncancelable so that climb isn't stopped halfway through
                        new ScheduleCommand(
                            Commands.sequence(
                                // Make sure / wait for climber to be fully extended,
                                PositionPIDCommand.generateCommand(
                                    swerve,
                                    Autos.flipIfNeeded(
                                        swerve,
                                        new Pose2d(
                                            climbPose,
                                            shouldFlip ? Rotation2d.fromDegrees(270.0) : Rotation2d.fromDegrees(90.0)
                                        )
                                    ),
                                    Seconds.of(10.0)
                                )
                                // Pull climber back down to move robot up
                            )
                        )
                    ).finallyDo(
                        (interrupted) -> {
                            // Put climber back down only if interrupted since the command got canceled on the way there
                        }
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
                    ).andThen(Commands.waitSeconds(outpostWaitTime));
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
