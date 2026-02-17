package com.spartronics4915.frc2026.subsystems;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;

import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem.IntakeState;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem.PivotState;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem.HoodClamp;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem.FeederState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem.IndexerState;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem.ShooterClamp;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    // Head mechanisms
    private final HoodSubsystem hood;
    private final TurretSubsystem turret;

    // Pipeline mechanisms
    private final FeederSubsystem feeder;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;

    // Other mechanisms
    private final ClimberSubsystem climber;
    private final IntakeSubsystem intake;
    private final PivotSubsystem pivot;

    // Bla bla bla bla 
    private RobotState currentRobotState;
    private Zone currentZone;
    private Zone previousZone;

    private boolean stateOverride;

    // Publishing for superstructure logging
    private final StringPublisher currentStatePublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("State").publish();
    private final StringPublisher currentZonePublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("Current Zone").publish();
    private final StringPublisher previousZonePublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("Previous State").publish();
    
    public Superstructure(
        HoodSubsystem hoodSubsystem,
        TurretSubsystem turretSubsystem,
        FeederSubsystem feederSubsystem,
        IndexerSubsystem indexerSubsystem,
        ShooterSubsystem shooterSubsystem,
        ClimberSubsystem climberSubsystem,
        IntakeSubsystem intakeSubsystem,
        PivotSubsystem pivotSubsystem
    ) {
        this.hood = hoodSubsystem;
        this.turret = turretSubsystem;
        this.feeder = feederSubsystem;
        this.indexer = indexerSubsystem;
        this.shooter = shooterSubsystem;
        this.climber = climberSubsystem;
        this.intake = intakeSubsystem;
        this.pivot = pivotSubsystem;

        this.currentZone = getCurrentZone();
        this.previousZone = currentZone;
    }

    private enum RobotState {
        TRAVERSAL,
        CRUISE,
        SHOOTING,
        CLIMB,
        IDLE,
        STOWED,
        TESTING
    }

    private enum PipelineState {
        ON(IndexerState.ON, FeederState.ON, ShooterClamp.UNRESTRICTED),
        OFF(IndexerState.OFF, FeederState.OFF, ShooterClamp.RESTRICTED);

        IndexerState indexerState;
        FeederState feederState;
        ShooterClamp shooterClamp;

        private PipelineState(
            IndexerState indexerState,
            FeederState feederState,
            ShooterClamp shooterClamp
        ) {
            this.indexerState = indexerState;
            this.feederState = feederState;
            this.shooterClamp = shooterClamp;
        }
    }

    private enum Zone {
        ALLIANCE_ZONE,
        TRENCH,
        BUMP,
        NEUTRAL_ZONE,
        OPPONENT_ZONE
    }

    @Override
    public void periodic() {
        currentZone = getCurrentZone();
        if (currentZone != previousZone) {
            stateOverride = false;
            previousZone = currentZone;
        }

        if (stateOverride != true) {
            switch (currentZone) {
                case ALLIANCE_ZONE:
                    if (currentRobotState == RobotState.SHOOTING) return;
                    switchState(RobotState.SHOOTING);
                    break;

                case TRENCH:
                    hood.setClamp(HoodClamp.RESTRICTED);
                    if (currentRobotState == RobotState.TRAVERSAL) return;
                    switchState(RobotState.TRAVERSAL);
                    break;

                case BUMP:
                    if (currentRobotState == RobotState.CRUISE) return;
                    switchState(RobotState.CRUISE);
                    break;

                case NEUTRAL_ZONE:
                    if (currentRobotState == RobotState.TRAVERSAL) return;
                    switchState(RobotState.TRAVERSAL);
                    break;

                case OPPONENT_ZONE:
                    if (currentRobotState == RobotState.CRUISE) return;
                    switchState(RobotState.CRUISE);
                    break;
            }
        } 

        currentStatePublisher.accept(currentRobotState.name());
        currentZonePublisher.accept(currentZone.name());
        previousZonePublisher.accept(previousZone.name());
    }

    private Zone getCurrentZone() {
        return Zone.ALLIANCE_ZONE;
    }

    public void setStateOverride(RobotState overrideState) {
        stateOverride = true;
        switchState(overrideState);
    }

    private Command setPipelineState(PipelineState state) {
        return Commands.sequence(
            shooter.setClampCommand(state.shooterClamp),
            Commands.waitUntil(() -> isShooterReady()),
            Commands.parallel(
                indexer.setStateCommand(state.indexerState),
                feeder.setStateCommand(state.feederState)
            )
        );
    }

    private void switchState(RobotState desiredState) {
        switch (desiredState) {
            case TRAVERSAL:
                transToTraversal();
                break;

            case CRUISE:
                transToCruise();
                break;

            case SHOOTING:
                transToShooting();
                break;

            case CLIMB:
                transToClimb();
                break;

            case IDLE:
                transToIdle();
                break;

            case STOWED:
                transToStowed();
                break;

            case TESTING:
                System.out.println("Chat, what are we doing?");
                break;
        }
        currentRobotState = desiredState;
    }

    //#region State Transitions

    private Command transToTraversal() {
        return Commands.sequence(
            pivot.setStateCommand(PivotState.READY),
            Commands.waitUntil(() -> isPivotSafe()),
            Commands.parallel(
                setPipelineState(PipelineState.OFF),
                intake.setStateCommand(IntakeState.ON)
            )
        );
    }

    private Command transToCruise() {
        return Commands.sequence(
            pivot.setStateCommand(PivotState.SAFE),
            Commands.waitUntil(() -> isPivotSafe()),
            Commands.parallel(
                setPipelineState(PipelineState.OFF),
                intake.setStateCommand(IntakeState.OFF)
            )
        );
    }

    private Command transToShooting() {
        return Commands.sequence(
            pivot.setStateCommand(PivotState.READY),
            Commands.waitUntil(() -> isPivotSafe()),
            Commands.parallel(
                setPipelineState(PipelineState.ON),
                intake.setStateCommand(IntakeState.ON)
            )
        );
    }

    private Command transToClimb() {
        return Commands.sequence(
            pivot.setStateCommand(PivotState.READY),
            Commands.waitUntil(() -> isPivotSafe()),
            Commands.parallel(
                setPipelineState(PipelineState.ON),
                intake.setStateCommand(IntakeState.OFF)
            )
        );
    }

    private Command transToIdle() {
        return Commands.sequence(
            pivot.setStateCommand(PivotState.READY),
            Commands.waitUntil(() -> isPivotSafe()),
            Commands.parallel(
                setPipelineState(PipelineState.OFF),
                intake.setStateCommand(IntakeState.OFF)
            )
        );
    }

    private Command transToStowed() {
        return Commands.sequence(
            setPipelineState(PipelineState.OFF),
            Commands.waitUntil(() -> isTurretSafe()),
            Commands.parallel(
                pivot.setStateCommand(PivotState.STOW),
                intake.setStateCommand(IntakeState.OFF)
            )
        );
    }

    //#endregion

    //#region Checks

    private boolean isShooterReady() {
        return shooter.getCurrentRPS() >= shooter.getCurrentSetpoint();
    }

    private boolean isPivotSafe() {
        return pivot.getPosition().getDegrees() 
            > PIVOT_SAFE_THRESHOLD.getDegrees();
    }

    private boolean isTurretSafe() {
        return (turret.getPosition().getDegrees() > TURRET_MIN_SAFE_THRESHOLD.getDegrees() 
            && turret.getPosition().getDegrees() < TURRET_MAX_SAFE_THRESHOLD.getDegrees()
        );
    }   

    //#endregion

}
