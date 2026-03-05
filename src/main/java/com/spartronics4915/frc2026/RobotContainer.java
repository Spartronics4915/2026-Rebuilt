// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.Constants.OperatorConstants;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.Constants.VisionConstants;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.trenchTransform;
import static com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem.isFieldRelative;
import static com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem.teleopHeadingOffset;

import com.spartronics4915.frc2026.commands.DriveCommand;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    public final VisionSubsystem visionSubsystem = new VisionSubsystem(
        VisionConstants.CameraConstants.cameras, 
        VisionConstants.LAYOUT, 
        new VisionConfiguration(), 
        swerveSubsystem::addVisionMeasurement, 
        swerveSubsystem
    );

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController debugController = new CommandXboxController(OperatorConstants.DEBUG_CONTROLLER_PORT);

    public DriveCommand driveCommand = new DriveCommand(driverController, debugController, swerveSubsystem);

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

        //#region Driver Controller

        ChassisSpeeds driverNudgeUp = new ChassisSpeeds(0.25, 0, 0);
        ChassisSpeeds driverNudgeLeft = new ChassisSpeeds(0, 0.25, 0);
        ChassisSpeeds driverNudgeRight = new ChassisSpeeds(0, -0.25, 0);
        ChassisSpeeds driverNudgeDown = new ChassisSpeeds(-0.25, 0, 0);

        driverController.povUp().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeUp);
            })
        );

        driverController.povLeft().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeLeft);
            })
        );

        driverController.povRight().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeRight);
            })
        );

        driverController.povDown().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeDown);
            })
        );

        driverController.leftBumper().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(
                    hubPose.minus(trenchTransform.times(swerveSubsystem.shouldFlip() ? -1 : 1)).getY()
                );
            })
        ).onFalse(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(0.0);
            })
        );

        driverController.rightBumper().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(
                    hubPose.plus(trenchTransform.times(swerveSubsystem.shouldFlip() ? -1 : 1)).getY()
                );
            })
        ).onFalse(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(0.0);
            })
        );

        driverController.a().onTrue(
            Commands.runOnce(() -> {
                teleopHeadingOffset = swerveSubsystem.getPose().getRotation();
            })
        );

        driverController.b().onTrue(
            Commands.runOnce(() -> {
                isFieldRelative = !isFieldRelative;
            })
        );

        driverController.leftTrigger().whileTrue(
            Commands.run(swerveSubsystem::lockModules, swerveSubsystem)
            .withName("X Brake Swerve")
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
