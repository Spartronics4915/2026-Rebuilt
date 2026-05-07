package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.control.AutoAimController;
import com.spartronics4915.frc2026.subsystems.mechanisms.*;
import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem.ClimberState;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem.IntakeState;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem.PivotState;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.*;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem.HoodClamp;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem.TurretClamp;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.*;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem.ShooterClamp;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A pure factory class that constructs complex multi-subsystem commands.
 */
public class SuperstructureCommands {

    public enum PipelineState {
        ON(IndexerSubsystem.IndexerState.FORWARD, FeederSubsystem.FeederState.FORWARD),
        OFF(IndexerSubsystem.IndexerState.OFF, FeederSubsystem.FeederState.OFF);

        final IndexerSubsystem.IndexerState indexerState;
        final FeederSubsystem.FeederState feederState;

        PipelineState(IndexerSubsystem.IndexerState indexer, FeederSubsystem.FeederState feeder) {
            this.indexerState = indexer;
            this.feederState = feeder;
        }
    }

    private final HoodSubsystem hood;
    private final TurretSubsystem turret;
    private final FeederSubsystem feeder;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final ClimberSubsystem climber;
    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;

    private final AutoAimController aimController;

    private PipelineState currentPipelineState;

    public SuperstructureCommands(
        HoodSubsystem hood, 
        TurretSubsystem turret, 
        FeederSubsystem feeder,
        IndexerSubsystem indexer, 
        ShooterSubsystem shooter, 
        ClimberSubsystem climber,
        IntakeSubsystem intake, 
        PivotSubsystem pivot, 
        AutoAimController aimController
    ) {
        this.hood = hood;
        this.turret = turret;
        this.feeder = feeder;
        this.indexer = indexer;
        this.shooter = shooter;
        this.climber = climber;
        this.intake = intake;
        this.pivot = pivot;
        this.aimController = aimController;
        this.currentPipelineState = PipelineState.OFF;
    }

    //#region Controls

    public Command setPipelineState(PipelineState state) {
        this.currentPipelineState = state;

        return Commands.defer(() -> {
            Command command = Commands.parallel(
                indexer.setStateCommand(state.indexerState),
                feeder.setStateCommand(state.feederState)
            );

            // If it's NOT off, wait for the shooter first; otherwise, just run the action
            return (state == PipelineState.OFF) ? command : command.beforeStarting(
                Commands.waitUntil(this::isShooterReady)
            );
        }, Set.of(indexer, feeder));
    }

    // !CLIMBER!
    // public Command setClimberState(ClimberState state) {
    //     return climber.setStateCommand(state);
    // }

    public Command resetDynamics() {
        return Commands.parallel(
            aimController.reset(),
            turret.setSetpointCommand(Rotation2d.fromDegrees(0)),
            hood.setSetpointCommand(Rotation2d.fromDegrees(0))
        );
    }

    public Command togglePipelineState() {
        return Commands.runOnce(() -> {
            currentPipelineState = currentPipelineState == PipelineState.ON
                ? PipelineState.OFF : PipelineState.ON;
            CommandScheduler.getInstance().schedule(setPipelineState(currentPipelineState));
        });
    }

    public Command intakeOn() {
        return Commands.parallel(
            pivot.setStateCommand(PivotState.READY),
            intake.setStateCommand(IntakeState.INTAKE)
        );
    }

    public Command intakeOff() {
        return Commands.parallel(
            pivot.setStateCommand(PivotState.SAFE),
            intake.setStateCommand(IntakeState.OFF)
        );
    }

    //#endregion
    //#region Conditionals

    /** Moves the pivot to READY only during autonomous; no-ops in teleop. */
    public Command conditionalPivotReady() {
        return Commands.either(
            pivot.setStateCommand(PivotSubsystem.PivotState.READY),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }

    public Command conditionalPivotSafe() {
        return Commands.either(
            pivot.setStateCommand(PivotSubsystem.PivotState.SAFE),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }
    
    /** Turns the intake ON only during autonomous; no-ops in teleop. */
    private Command conditionalIntakeOn() {
        return Commands.either(
            intake.setStateCommand(IntakeSubsystem.IntakeState.INTAKE),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }

    /** Turns auto-aim ON only during autonomous; no-ops in teleop. Aim is always on in auto. */
    private Command conditionalAutoAimOn() {
        return Commands.either(
            aimController.setAimState(true),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }

    /** Turns auto-aim OFF only during autonomous; no-ops in teleop. Aim is always on in auto. */
    private Command conditionalAutoAimOff() {
        return Commands.either(
            aimController.setAimState(false),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }

    /** Turns auto-shoot ON only during autonomous; no-ops in teleop. */
    private Command conditionalAutoShootOn() {
        return Commands.either(
            aimController.setShootingState(true),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }

    /** Turns auto-shoot OFF only during autonomous; no-ops in teleop. Aim remains ON. */
    private Command conditionalAutoShootOff() {
        return Commands.either(
            aimController.setShootingState(false),
            Commands.none(),
            DriverStation::isAutonomous
        );
    }

    //#endregion
    //#region State Commands

    public Command traversal() {
        return Commands.sequence(
            Commands.parallel(
                conditionalPivotReady(),
                conditionalAutoAimOn(),
                conditionalAutoShootOff(),
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                conditionalIntakeOn()
                // climber.setStateCommand(ClimberState.DOWN) // !CLIMBER!
            ),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED)
            )
        );
    }

    public Command trench() {
        return Commands.sequence(
            Commands.parallel(
                conditionalPivotReady(),
                conditionalAutoAimOn(),
                conditionalAutoShootOff(),
                hood.setClampCommand(HoodClamp.RESTRICTED),
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                conditionalIntakeOn()
                // climber.setStateCommand(ClimberState.DOWN) // !CLIMBER!
            ),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED)
            )
        );
    }

    public Command tower() {
        return Commands.sequence(
            Commands.parallel(
                conditionalPivotReady(),
                conditionalAutoAimOn(),
                conditionalAutoShootOff(),
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                conditionalIntakeOn()
                // climber.setStateCommand(ClimberState.DOWN) // !CLIMBER!
            ),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED)
            )
        );
    }

    public Command cruise() {
        return Commands.sequence(
            Commands.parallel(
                conditionalPivotReady(),
                conditionalAutoAimOn(),
                conditionalAutoShootOff(),
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                conditionalIntakeOn()
                // climber.setStateCommand(ClimberState.DOWN) // !CLIMBER!
            ),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED)
            )
        );
    }

    public Command shooting() {
        return Commands.sequence(
            Commands.parallel(
                conditionalPivotReady(),
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                conditionalAutoAimOn(),
                conditionalAutoShootOn(),
                conditionalIntakeOn()
                // climber.setStateCommand(ClimberState.DOWN) // !CLIMBER!
            ),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED)
            )
        );
    }

    public Command climb() {
        return Commands.sequence(
            pivot.setStateCommand(PivotSubsystem.PivotState.READY),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                conditionalAutoAimOff(),
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    public Command idle() {
        return Commands.sequence(
            Commands.parallel(
                conditionalAutoAimOn(),
                conditionalAutoShootOff(),
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                pivot.setStateCommand(PivotSubsystem.PivotState.READY)
                // climber.setStateCommand(ClimberState.DOWN) // !CLIMBER!
            ),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    public Command stowed() {
        return Commands.sequence(
            conditionalAutoAimOff(),
            conditionalAutoShootOff(),
            turret.setClampCommand(TurretClamp.RESTRICTED),
            Commands.waitUntil(this::isTurretSafe),
            pivot.setStateCommand(PivotSubsystem.PivotState.STOW),
            Commands.parallel(
                hood.setClampCommand(HoodClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                // climber.setStateCommand(ClimberState.DOWN), // !CLIMBER!
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    //#endregion
    //#region Guards

    public boolean isShooterReady() {
        double setpoint = shooter.getCurrentSetpoint();
        return Math.abs(shooter.getCurrentRPS() - setpoint) < shooterReadyThresholdRPS
            && setpoint != 0;
    }

    private boolean isPivotSafe() {
        return (Robot.isReal() ? pivot.getPosition() : pivot.getSetpoint()).getDegrees() <= pivotSafeThreshold.getDegrees();
    }

    private boolean isTurretSafe() {
        double degrees = turret.getPosition().getDegrees();
        return degrees >= turretMinSafeThreshold.getDegrees() 
            && degrees <= turretMaxSafeThreshold.getDegrees();
    }

    //#endregion

}
