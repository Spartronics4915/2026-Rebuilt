// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.Constants.OperatorConstants;
import com.spartronics4915.frc2026.autos.Autos;

import static com.spartronics4915.frc2026.Constants.VisionConstants.VisionState.*;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;

import com.spartronics4915.frc2026.commands.DriveCommand;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

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
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem(
        swerveSubsystem::addVisionMeasurement,
        () -> swerveSubsystem.getPose(),
        () -> swerveSubsystem.getPastVisionPose(VisionSubsystem.poseTimestamp),
        () -> swerveSubsystem.getFieldVelocity(),
        () -> swerveSubsystem.getHeading()
    );

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

        driverController.y().onTrue(
            Commands.runOnce(() -> {
                VisionSubsystem.visionState = (VisionSubsystem.visionState == GLOBAL) ? LOCAL : GLOBAL;
                if (VisionSubsystem.visionState == LOCAL) {
                    if (SwerveSubsystem.isRightAlliance == true) VisionSubsystem.setLocalCamera("daniil");
                        else VisionSubsystem.setLocalCamera("daniil");
                }
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
