package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;

import com.spartronics4915.frc2026.subsystems.control.AutoAimController;
import com.spartronics4915.frc2026.subsystems.mechanisms.*;
import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem.ClimberState;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem.IntakeState;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem.PivotState;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.*;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem.HoodClamp;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem.TurretClamp;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.*;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem.FeederState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem.IndexerState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem.ShooterClamp;

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
        return Commands.sequence(
            Commands.waitUntil(() -> isShooterReady()),
            Commands.parallel(
                indexer.setStateCommand(state.indexerState),
                feeder.setStateCommand(state.feederState)
            )
        );
    }

    public Command setClimberState(ClimberState state) {
        return climber.setStateCommand(state);
    }

    public Command resetDynamics() {
        return aimController.reset();
    }

    public Command togglePipelineState() {
        return Commands.runOnce(() -> {
            currentPipelineState = currentPipelineState == PipelineState.ON
                ? PipelineState.OFF : PipelineState.ON;
            CommandScheduler.getInstance().schedule(setPipelineState(currentPipelineState));
        });
    }

    public Command intakeOn() {
        return Commands.sequence(
            Commands.parallel(
                pivot.setStateCommand(PivotState.READY),
                intake.setStateCommand(IntakeState.INTAKE)
            )
        );
    }

    public Command intakeOff() {
        return Commands.sequence(
            Commands.parallel(
                pivot.setStateCommand(PivotState.SAFE),
                intake.setStateCommand(IntakeState.OFF)
            )
        );
    }

    //#endregion
    //#region Conditionals

    /** Moves the pivot to READY only during autonomous; no-ops in teleop. */
    private Command conditionalPivotReady() {
        return Commands.either(
            pivot.setStateCommand(PivotSubsystem.PivotState.READY),
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

    //#endregion
    //#region State Commands

    public Command traversal() {
        return Commands.sequence(
            Commands.parallel(
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                conditionalPivotReady(),
                climber.setStateCommand(ClimberState.DOWN)),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                conditionalIntakeOn()
            )
        );
    }

    /** Specifically for the Trench zone to ensure the hood is clamped */
    public Command trenchTraversal() {
        return Commands.sequence(
            hood.setClampCommand(HoodClamp.RESTRICTED),
            hood.setSetpointCommand(Rotation2d.fromDegrees(0)),
            traversal()
        );
    }

    public Command cruise() {
        return Commands.sequence(
            Commands.parallel(
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                pivot.setStateCommand(PivotSubsystem.PivotState.SAFE),
                climber.setStateCommand(ClimberState.DOWN)),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    public Command shooting() {
        return Commands.sequence(
            Commands.parallel(
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                conditionalPivotReady(),
                climber.setStateCommand(ClimberState.DOWN)),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                conditionalIntakeOn()
            )
        );
    }

    public Command climb() {
        return Commands.sequence(
            pivot.setStateCommand(PivotSubsystem.PivotState.READY),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                shooter.setClampCommand(ShooterClamp.UNRESTRICTED),
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    public Command idle() {
        return Commands.sequence(
            Commands.parallel(
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                pivot.setStateCommand(PivotSubsystem.PivotState.READY),
                climber.setStateCommand(ClimberState.DOWN)),
            Commands.waitUntil(this::isPivotSafe),
            Commands.parallel(
                turret.setClampCommand(TurretClamp.UNRESTRICTED),
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    public Command stowed() {
        return Commands.sequence(
            turret.setClampCommand(TurretClamp.RESTRICTED),
            Commands.waitUntil(this::isTurretSafe),
            pivot.setStateCommand(PivotSubsystem.PivotState.STOW),
            Commands.parallel(
                shooter.setClampCommand(ShooterClamp.RESTRICTED),
                climber.setStateCommand(ClimberState.DOWN),
                intake.setStateCommand(IntakeSubsystem.IntakeState.OFF)
            )
        );
    }

    //#endregion
    //#region Guards

    // --- Guard Conditions ---

    public boolean isShooterReady() {
        return shooter.getCurrentRPS() >= shooter.getCurrentSetpoint() 
            && shooter.getCurrentSetpoint() != 0;
    }

    private boolean isPivotSafe() {
        return pivot.getPosition().getDegrees() <= PIVOT_SAFE_THRESHOLD.getDegrees();
    }

    private boolean isTurretSafe() {
        double degrees = turret.getPosition().getDegrees();
        return degrees >= TURRET_MIN_SAFE_THRESHOLD.getDegrees() 
            && degrees <= TURRET_MAX_SAFE_THRESHOLD.getDegrees();
    }

    //#endregion

}
