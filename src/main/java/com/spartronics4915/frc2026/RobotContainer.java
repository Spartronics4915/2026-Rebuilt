// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.Constants.OperatorConstants;
import com.spartronics4915.frc2026.autos.Autos;
import com.spartronics4915.frc2026.autos.ComplexAutoChooser;
import com.spartronics4915.frc2026.autos.DriveToPOI;
import com.spartronics4915.frc2026.autos.NeutralZoneAutos;
import com.spartronics4915.frc2026.autos.DriveToPOI.POI;
import com.spartronics4915.frc2026.autos.ZoneTransition;
import com.spartronics4915.frc2026.autos.ZoneTransition.TraversalMethod;
import com.spartronics4915.frc2026.Constants.VisionConstants;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.trenchTransform;
import static com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem.isFieldRelative;
import static com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem.teleopHeadingOffset;

import java.util.Set;

import com.spartronics4915.frc2026.commands.DriveCommand;
import com.spartronics4915.frc2026.subsystems.Superstructure;
import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem.FeederState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem.IndexerState;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public final TurretSubsystem turretSubsystem = new TurretSubsystem();

    public final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public final FeederSubsystem feederSubsystem = new FeederSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(SwerveConfigurations.COMP_CHASSIS);
    public final VisionSubsystem visionSubsystem = new VisionSubsystem(
        VisionConstants.CameraConstants.cameras, 
        VisionConstants.LAYOUT, 
        new VisionConfiguration(), 
        swerveSubsystem::addVisionMeasurement, 
        swerveSubsystem
    );

    public final Superstructure superstructure = new Superstructure(
        hoodSubsystem,
        turretSubsystem,
        feederSubsystem,
        indexerSubsystem,
        shooterSubsystem,
        climberSubsystem,
        intakeSubsystem,
        pivotSubsystem,
        swerveSubsystem
    );
    
    private final ZoneTransition transitionFactory = new ZoneTransition(swerveSubsystem, visionSubsystem);
    private final DriveToPOI POIFactory = new DriveToPOI(swerveSubsystem);
    private final NeutralZoneAutos neutralZoneFactory = new NeutralZoneAutos(swerveSubsystem);

    private final ComplexAutoChooser autoChooser = new ComplexAutoChooser(transitionFactory, POIFactory, neutralZoneFactory, 10);

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    public DriveCommand driveCommand = new DriveCommand(driverController, swerveSubsystem);

    public RobotContainer() {
        configureBindings();

        SmartDashboard.putData(
            "Pipeline On", 
            Commands.parallel(
                feederSubsystem.setStateCommand(FeederState.ON),
                indexerSubsystem.setStateCommand(IndexerState.ON)
            )
        );

        SmartDashboard.putData(
            "Pipeline Off", 
            Commands.parallel(
                feederSubsystem.setStateCommand(FeederState.OFF),
                indexerSubsystem.setStateCommand(IndexerState.OFF)
            )
        );
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
                isFieldRelative = !isFieldRelative;
            })
        );

        driverController.a().onTrue(
            Commands.runOnce(() -> {
                teleopHeadingOffset = swerveSubsystem.getPose().getRotation();
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

        driverController.povUp().whileTrue(
            POIFactory.generateCommand(POI.DEPOT)
        );

        driverController.povDown().whileTrue(
            POIFactory.generateCommand(POI.OUTPOST)
        );
        
        driverController.y().whileTrue(
            POIFactory.generateCommand(POI.TOWER)
        );

        driverController.leftTrigger().onTrue(
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

        driverController.rightTrigger().onTrue(
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
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.defer(() -> autoChooser.getAuto(), Set.of(swerveSubsystem));
    }
}
