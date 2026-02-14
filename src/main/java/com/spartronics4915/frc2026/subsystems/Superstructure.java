package com.spartronics4915.frc2026.subsystems;

import com.spartronics4915.frc2026.Robot;

import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.FeederSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.IndexerSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import static com.spartronics4915.frc2026.subsystems.Superstructure.SuperState.*;
import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{

    private final FeederSubsystem feeder;
    private final HoodSubsystem hood;
    private final IndexerSubsystem indexer;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final ClimberSubsystem climber;
    private final PivotSubsystem pivot;
    private final SwerveSubsystem swerve;

    private SuperState currentState;
    private RobotLocation currentLocation;
    private RobotLocation lastLocation;

    private boolean override;

    public Superstructure(
        FeederSubsystem feederSubsystem,
        HoodSubsystem hoodSubsystem,
        IndexerSubsystem indexerSubsystem,
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        TurretSubsystem turretSubsystem,
        ClimberSubsystem climberSubsystem,
        PivotSubsystem pivotSubsystem,
        SwerveSubsystem swerveSubsystem
    ) {
        this.feeder = feederSubsystem;
        this.hood = hoodSubsystem;
        this.indexer = indexerSubsystem;
        this.intake = intakeSubsystem;
        this.shooter = shooterSubsystem;
        this.turret = turretSubsystem;
        this.climber = climberSubsystem;
        this.pivot = pivotSubsystem;
        this.swerve = swerveSubsystem;

        this.currentLocation = getCurrentLocation();
        this.lastLocation = currentLocation;
    }
    
    public enum SuperState {
        STOWED,
        IDLE, 
        TRAVERSAL,
        SHOOTING,
        SAFE,
        PRE_CLIMB,
        ACTIVE_CLIMB,
        TESTING
    }

    public enum RobotLocation {
        ALLIANCE_ZONE,
        TRENCH,
        BUMP,
        NEUTRAL_ZONE,
        OTHER_ALLIANCE_ZONE,
        THE_VOID
    }

    @Override
    public void periodic() {
        currentLocation = getCurrentLocation();
        if (currentLocation != lastLocation) override = false;
        if (override) return;

        switch (currentLocation) {
            case ALLIANCE_ZONE:
                if (!Robot.hubEnabled || Robot.timeUntilSwitch > 3) return;
                if (currentState == SHOOTING) return;
                transition(SHOOTING);
                break;

            case TRENCH:
                if (currentState == SAFE) return;
                transition(SAFE);
                break;

            case BUMP:
                if (currentState == SAFE) return;
                transition(SAFE);
                break;

            case NEUTRAL_ZONE:
                if (currentState == TRAVERSAL) return;
                transition(TRAVERSAL);
                break;

            case OTHER_ALLIANCE_ZONE:
                if (currentState == SAFE) return;
                transition(SAFE);
                break;
            
            case THE_VOID:
                if (currentState == IDLE) return;
                transition(IDLE);
                break;
        }
    }

    private RobotLocation getCurrentLocation() {
        Pose2d currentPose = swerve.getRobotPose().plus(turretTransform);
        // Do all the fancy stuff to get the location
        // Check if the last location is the same as this location for override purposes
        return RobotLocation.ALLIANCE_ZONE;
    }

    private void transition(SuperState wantedState) {
        switch (wantedState) {
            case STOWED:
                toStowed();
                break;
            
            case IDLE:
                toIdle();
                break;

            case TRAVERSAL:
                toTransversal();
                break;

            case SHOOTING:
                toShooting();
                break;

            case SAFE:
                toSafe();
                break;

            case PRE_CLIMB:
                toPreClimb();
                break;

            case ACTIVE_CLIMB:
                toActiveClimb();
                break;

            case TESTING:
                break;
        }
    }

    //#endregion

    //#region Transitions

    private Command toStowed() {
        return null;
    }

    private Command toIdle() { 
        return null;
    }

    private Command toTransversal() { 
        return null;
    }
    
    private Command toShooting() { 
        return null;
    }

    private Command toSafe() { 
        return null;
    }

    private Command toPreClimb() { 
        return null;
    }

    private Command toActiveClimb() { 
        return null;
    }

    //#endregion

    //#region Checks

    private boolean isPivotSafe() {
        return pivot.getPosition().getRotations() > Rotation2d.fromDegrees(0).getRotations();
    }
    
    private boolean isHoodSafe() {
        return hood.getPosition().getRotations() > Rotation2d.fromDegrees(0).getRotations();
    }

    private boolean isTurretSafe() {
        return true; //turret.getPosition().getRotations() > Rotation2d.fromDegrees(0).getRotations();
    }

    private boolean isShooterReady() {
        return true; // This should check if the shooter is at or really close to the set point
    }

    //#endregion

    public void setStateOverride(SuperState overrideState) {
        override = true;
        transition(overrideState);
    }

}

