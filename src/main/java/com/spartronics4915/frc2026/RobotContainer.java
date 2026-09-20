// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.Constants.AutoAimConstants;
import com.spartronics4915.frc2026.Constants.OperatorConstants;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.autos.Autos;
import com.spartronics4915.frc2026.autos.ComplexAutoChooser;
import com.spartronics4915.frc2026.autos.DriveToPOI;
import com.spartronics4915.frc2026.autos.NeutralZoneAutos;
import com.spartronics4915.frc2026.autos.PreAlignment;
import com.spartronics4915.frc2026.autos.ZoneTransition;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.trenchTransform;

import java.util.Set;

import com.spartronics4915.frc2026.commands.DriveCommand;
import com.spartronics4915.frc2026.commands.SuperstructureCommands;
import com.spartronics4915.frc2026.commands.SuperstructureCommands.PipelineState;
import com.spartronics4915.frc2026.subsystems.control.AutoAimController;
import com.spartronics4915.frc2026.subsystems.control.AutoAimController.ManualOverride;
import com.spartronics4915.frc2026.subsystems.control.Superstructure;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem.IntakeState;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem.FeederState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem.IndexerState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2026.util.simulation.FuelSimHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and triggers) should be declared here.
 */
public class RobotContainer {

    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public final TurretSubsystem turretSubsystem = new TurretSubsystem();

    public final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // public final ClimberSubsystem climberSubsystem = new ClimberSubsystem(); // !CLIMBER!

    public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
    public final FeederSubsystem feederSubsystem = new FeederSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    
    public final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance(SwerveConfigurations.COMP_CHASSIS);
    public final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance(swerveSubsystem);
    
    private final ZoneTransition transitionFactory = new ZoneTransition(swerveSubsystem, visionSubsystem);
    private final DriveToPOI POIFactory = new DriveToPOI(swerveSubsystem, null); // !CLIMBER!
    private final NeutralZoneAutos neutralZoneFactory = new NeutralZoneAutos(swerveSubsystem);
    private final PreAlignment preAlignmentFactory = new PreAlignment(swerveSubsystem);

    private final AutoAimController autoAimController = new AutoAimController(
        hoodSubsystem,
        turretSubsystem,
        swerveSubsystem,
        shooterSubsystem
    );

    private final FuelSimHandler fuelSimHandler = new FuelSimHandler(swerveSubsystem, intakeSubsystem, autoAimController);

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
    private final CommandXboxController debugController = new CommandXboxController(OperatorConstants.DEBUG_CONTROLLER_PORT);

    public DriveCommand driveCommand = new DriveCommand(driverController, debugController, swerveSubsystem);
    public SuperstructureCommands superstructureCommands = new SuperstructureCommands(
        hoodSubsystem, 
        turretSubsystem, 
        feederSubsystem, 
        indexerSubsystem, 
        shooterSubsystem, 
        null, // !CLIMBER!
        intakeSubsystem,
        pivotSubsystem, 
        autoAimController
    );

    public final Superstructure superstructure = new Superstructure(
        swerveSubsystem, 
        autoAimController, 
        superstructureCommands
    );

    private final ComplexAutoChooser autoChooser = new ComplexAutoChooser(transitionFactory, POIFactory, neutralZoneFactory, preAlignmentFactory, superstructure, 20);

    public RobotContainer() {
        configureBindings();

        // Preserve turret history so a camera frame uses its capture-time transform.
        visionSubsystem.recordTurretAngle(
            turretSubsystem.getPosition(),
            Timer.getFPGATimestamp());
        turretSubsystem.setVisionObserver(visionSubsystem::recordTurretAngle);
        visionSubsystem.configureDefaultCameras();

        // Initialize hood with turret reference for 3D visualization
        hoodSubsystem.setTurretSubsystem(turretSubsystem);

        feederSubsystem.setDistanceSupplier(autoAimController::getDistanceToTarget);

        SmartDashboard.putData("Auto-Aim Toggle", autoAimController.aimToggle());
        SmartDashboard.putData("Auto-Shoot Toggle", autoAimController.shootingToggle());
        SmartDashboard.putData("Reset Dynamics", superstructureCommands.resetDynamics());
        SmartDashboard.putData("Fuel Sim/Reset", Commands.runOnce(fuelSimHandler::resetSimulatedFuel));
        SmartDashboard.putData("Fuel Sim/Add Fuel", Commands.runOnce(fuelSimHandler::intakeSimulatedFuel));
        SmartDashboard.putData("Pipeline On", superstructureCommands.setPipelineState(PipelineState.ON));
        SmartDashboard.putData("Pipeline Off", superstructureCommands.setPipelineState(PipelineState.OFF));
        SmartDashboard.putData("Reset Odometry", Commands.runOnce(
            () -> swerveSubsystem.resetPose(visionSubsystem.getVisionPose())
        ));
        SmartDashboard.putData("Temp Waypoint", Commands.defer(
            () -> Autos.generatePathFromWaypoint(swerveSubsystem, new Translation2d(2.5, 2.0), Rotation2d.kZero),
            Set.of(swerveSubsystem)
        ));
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

        // !CLIMBER! (remove the conflicting nudge commands)
        // driverController.povUp().onTrue(
        //     climberSubsystem.setStateCommand(ClimberState.JORBIT)
        // );

        // driverController.povDown().onTrue(
        //     climberSubsystem.setStateCommand(ClimberState.DOWN)
        // );

        driverController.povUp().whileTrue(
            Commands.run(() -> swerveSubsystem.drive(driverNudgeUp))
        );

        driverController.povLeft().whileTrue(
            Commands.run(() -> swerveSubsystem.drive(driverNudgeLeft), swerveSubsystem)
        );

        driverController.povRight().whileTrue(
            Commands.run(() -> swerveSubsystem.drive(driverNudgeRight), swerveSubsystem)
        );

        driverController.povDown().whileTrue(
            Commands.run(() -> swerveSubsystem.drive(driverNudgeDown))
        );

        driverController.leftBumper().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(
                    hubPose.minus(trenchTransform).getY()
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
                    hubPose.plus(trenchTransform).getY()
                );
            })
        ).onFalse(
            Commands.runOnce(() -> {
                swerveSubsystem.setMovementOverride(0.0);
            })
        );

        driverController.a().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.resetHeadingOffset();
            })
        );

        driverController.b().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.toggleFieldRelative();
            })
        );

        // !CLIMBER!
        // driverController.y().whileTrue(
        //     POIFactory.generateCommand(POI.TOWER)
        // );

        driverController.leftTrigger().whileTrue(
            Commands.run(swerveSubsystem::lockModules, swerveSubsystem)
            .withName("X Brake Swerve")
        );

        //driverController.start().onTrue(
        //    Commands.runOnce(
        //        () -> swerveSubsystem.resetPose(visionSubsystem.getVisionPose())
        //    )
        //);

        //#endregion
        //#region Operator Controller

        operatorController.povUp().whileTrue(
            Commands.run(() -> {
                pivotSubsystem.deltaSetpoint(Rotation2d.fromDegrees(1.5));
            })
        );

        operatorController.povLeft().whileTrue(
            autoAimController.setManualOverride(ManualOverride.LEFT_CORNER)
        );

        operatorController.povRight().whileTrue(
            autoAimController.setManualOverride(ManualOverride.RIGHT_CORNER)
        );

        operatorController.povDown().whileTrue(
            Commands.run(() -> {
                pivotSubsystem.deltaSetpoint(Rotation2d.fromDegrees(-1.5));
            })
        );

        operatorController.leftStick().onTrue(
            Commands.runOnce(() -> {
                pivotSubsystem.resetMechanism(Rotation2d.kZero);
            })
        );

        operatorController.rightStick().onTrue(
            Commands.runOnce(() -> {
                pivotSubsystem.resetMechanism(Rotation2d.kZero);
            })
        );

        SmartDashboard.putData("Pivot Reset", Commands.runOnce(() -> pivotSubsystem.resetMechanism(Rotation2d.kZero)));

        operatorController.leftTrigger().onTrue(
            Commands.parallel(
                superstructureCommands.intakeOn(),
                superstructure.getReturnToZoneCommand()
            )
        ).onFalse(
            superstructureCommands.intakeOff()
        );

        operatorController.rightTrigger().whileTrue(
            autoAimController.overrideShootCommand()
        );

        operatorController.leftBumper().whileTrue(
            autoAimController.overrideTargetCommand(
                AutoAimConstants.leftPassTarget
            )
        );

        operatorController.rightBumper().whileTrue(
            autoAimController.overrideTargetCommand(
                AutoAimConstants.rightPassTarget
            )
        );

        operatorController.a().onTrue(
            intakeSubsystem.setStateCommand(IntakeState.OUTTAKE)
        ).onFalse(
            intakeSubsystem.setStateCommand(IntakeState.OFF)
        );

        operatorController.b().onTrue(
            Commands.parallel(
                feederSubsystem.setStateCommand(FeederState.REVERSE),
                indexerSubsystem.setStateCommand(IndexerState.REVERSE)
            )
        ).onFalse(
            Commands.parallel(
                feederSubsystem.setStateCommand(FeederState.OFF),
                indexerSubsystem.setStateCommand(IndexerState.OFF)
            )
        );

        operatorController.x().onTrue(
            autoAimController.shootingToggle()
        );

        operatorController.y().onTrue(
            superstructureCommands.stowed()
        );

        operatorController.start().onTrue(
            autoAimController.aimToggle()
        );

        operatorController.back().onTrue(
            superstructureCommands.resetDynamics()
        );

        //#endregion
        //#region Debug Controller

        // Driver nudge defs are in the driverController section

        /*
            Left trigger: Intake
            Right trigger: Shoot

            Left bumper: Reverse pipeline
            Right bumper: Intake eject

            Left stick: Translate
            Right stick: Rotate

            X: X-Brake
            Y: 
            B: Reset heading
            A: 

            back: Reset dynamics 
            start: Toggle auto-aim

            Left stick press:
            Right stick press:

            Pov-up: (Trench presets?) up and down?
            Pov-down:
            Pov-right: Shoot preset right
            Pov-left: Shoot preset left
        */

        // Triggers:

        debugController.leftTrigger().onTrue(
            Commands.parallel(
                superstructureCommands.intakeOn(),
                superstructure.getReturnToZoneCommand())
        ).onFalse(
            superstructureCommands.intakeOff()
        );

        debugController.rightTrigger().whileTrue(
            autoAimController.overrideShootCommand()
        );

        // Bumpers:

        debugController.leftBumper().onTrue(
            Commands.parallel(
                feederSubsystem.setStateCommand(FeederState.REVERSE),
                indexerSubsystem.setStateCommand(IndexerState.REVERSE)
            )
        ).onFalse(
            Commands.parallel(
                feederSubsystem.setStateCommand(FeederState.OFF),
                indexerSubsystem.setStateCommand(IndexerState.OFF)
            )
        );

        debugController.rightBumper().onTrue(
            intakeSubsystem.setStateCommand(IntakeState.OUTTAKE)
        ).onFalse(
            intakeSubsystem.setStateCommand(IntakeState.OFF)
        );

        // Buttons:

        debugController.x().whileTrue(
            Commands.run(swerveSubsystem::lockModules, swerveSubsystem)
                .withName("X Brake Swerve")
        );

        debugController.b().onTrue(
            Commands.runOnce(() -> {
                swerveSubsystem.resetHeadingOffset();
            })
        );

        debugController.start().onTrue(
            autoAimController.aimToggle()
        );

        debugController.back().onTrue(
            superstructureCommands.resetDynamics()
        );

        // D-Pad:

        debugController.povUp().whileTrue(
            autoAimController.setManualOverride(ManualOverride.LEFT_CORNER)
        );

        debugController.povLeft().whileTrue(
            autoAimController.setManualOverride(ManualOverride.LEFT_TRENCH)
        );

        debugController.povRight().whileTrue(
            autoAimController.setManualOverride(ManualOverride.RIGHT_TRENCH)
        );

        debugController.povDown().whileTrue(
            autoAimController.setManualOverride(ManualOverride.RIGHT_CORNER)
        );

        //#endregion

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
