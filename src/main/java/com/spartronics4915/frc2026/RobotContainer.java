// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.Constants.OperatorConstants;
import com.spartronics4915.frc2026.autos.Autos;
import com.spartronics4915.frc2026.autos.ZoneTransition;
import com.spartronics4915.frc2026.autos.ZoneTransition.TraversalMethod;
import com.spartronics4915.frc2026.Constants.VisionConstants;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.trenchTransform;

import com.spartronics4915.frc2026.commands.DriveCommand;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(SwerveConfigurations.COMP_CHASSIS);
    private final VisionSubsystem visionSubsystem = VisionSubsystem.builder()
        .addCamera("daniil", VisionConstants.RIGHT_PROCESSOR)
        //.addCamera("evan", VisionConstants.LEFT_PROCESSOR)
        .setFieldLayout(VisionConstants.LAYOUT)
        .setSimPoseSupplier(() -> swerveSubsystem.getRobotPose())
        .setRobotVelocitySupplier(() -> swerveSubsystem.getFieldVelocity())
        .setUsedPoseSupplier(() -> swerveSubsystem.getPastVisionPose(VisionSubsystem.getPoseTimestamp()))
        .setPoseConsumer((pose, time, stdDevs) -> {
            swerveSubsystem.addVisionMeasurement(pose, time, stdDevs);
        })
        .setConfiguration(new VisionConfiguration())
        .build();
    private final ZoneTransition transitionFactory = new ZoneTransition(swerveSubsystem, visionSubsystem);

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public DriveCommand driveCommand = new DriveCommand(driverController, swerveSubsystem);

    public RobotContainer() {
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories
     * in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(driveCommand);

        driverController.b().onTrue(
            Commands.runOnce(() -> {
                IS_FIELD_RELATIVE = !IS_FIELD_RELATIVE;
            })
        );

        driverController.a().onTrue(
            Commands.runOnce(() -> {
                TELEOP_HEADING_OFFSET = swerveSubsystem.getPose().getRotation();
            })
        );

        driverController.leftBumper().whileTrue(
            transitionFactory.generateCommand(TraversalMethod.LEFT_BUMP)
        );

        driverController.rightBumper().whileTrue(
            transitionFactory.generateCommand(TraversalMethod.RIGHT_BUMP)
        );

        driverController.povLeft().whileTrue(
            transitionFactory.generateCommand(TraversalMethod.LEFT_TRENCH)
        );

        driverController.povRight().whileTrue(
            transitionFactory.generateCommand(TraversalMethod.RIGHT_TRENCH)
        ); 

        driverController.leftTrigger().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(
                    new Pose2d(
                        0,
                        hubPose.plus(trenchTransform).getY(),
                        Rotation2d.fromDegrees(0)
                    )
                );
            })
        ).onFalse(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(null);
            })
        );

        driverController.rightTrigger().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(
                    new Pose2d(
                        0,
                        hubPose.minus(trenchTransform).getY(),
                        Rotation2d.fromDegrees(0)
                    )
                );
            })
        ).onFalse(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(null);
            })
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autos.nothingAuto();
    }
}
