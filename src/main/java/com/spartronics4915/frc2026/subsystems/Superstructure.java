package com.spartronics4915.frc2026.subsystems;

import com.spartronics4915.frc2026.subsystems.mechanisms.path.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.FeederSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.IndexerSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.path.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.ClimberSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.PivotSubsystem;

import static com.spartronics4915.frc2026.subsystems.Superstructure.SuperState.*;
import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{

    private final FeederSubsystem feeder;
    private final HoodSubsystem hood;
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem spindexer;
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
        IntakeSubsystem intakeSubsystem,
        ShooterSubsystem shooterSubsystem,
        IndexerSubsystem spindexerSubsystem,
        TurretSubsystem turretSubsystem,
        ClimberSubsystem climberSubsystem,
        PivotSubsystem pivotSubsystem,
        SwerveSubsystem swerveSubsystem
    ) {
        this.feeder = feederSubsystem;
        this.hood = hoodSubsystem;
        this.intake = intakeSubsystem;
        this.shooter = shooterSubsystem;
        this.spindexer = spindexerSubsystem;
        this.turret = turretSubsystem;
        this.climber = climberSubsystem;
        this.pivot = pivotSubsystem;
        this.swerve = swerveSubsystem;

        this.currentState = STOWED;
        this.currentLocation = getCurrentLocation();
        this.lastLocation = currentLocation;
    }
    
    public enum SuperState {
        TRAVERSAL,
        SHOOTING,
        TRENCH,
        PRE_CLIMB,
        ACTIVE_CLIMB,
        STOWED,
        IDLE
    }

    public enum RobotLocation {
        ALLIANCE_ZONE,
        TRENCH,
        BUMP,
        NEUTRAL_ZONE
    }

    // There should be an override something that will make it 
    // so the operator can override the current state until the robot enters a new region

    @Override
    public void periodic() {
        RobotLocation currentLocation = getCurrentLocation();
        if (currentLocation != lastLocation) override = false;
        if (override) return;

        switch (currentLocation) {
            case ALLIANCE_ZONE: 
                if (currentState == SHOOTING) return;
                transition(SHOOTING);
                break;
            
            case TRENCH:
                if (currentState == TRENCH) return;
                transition(TRENCH);
                break;
            
            // Do we even need bump as a separate location?
            case BUMP:
                if (currentState == TRAVERSAL) return;
                transition(TRAVERSAL);
                break;

            case NEUTRAL_ZONE:
                if (currentState == TRAVERSAL) return;
                transition(TRAVERSAL);
                break;
            
            default:
                if (currentState == IDLE) return;
                transition(IDLE);
                break;
            
        }
    }

    /**
     * 
     */
    private RobotLocation getCurrentLocation() {
        Pose2d currentPose = swerve.getRobotPose().plus(turretTransform);
        // Do all the fancy stuff to get the location
        // Check if the last location is the same as this location for override purposes
        return RobotLocation.ALLIANCE_ZONE;
    }

    /**
     * 
     */
    private void transition(SuperState wantedState) {
        switch (wantedState) {
            case TRAVERSAL:
                transitionToTraversal();
                break;

            case SHOOTING:
                transitionToShooting();
                break;

            case TRENCH:
                transitionToTrench();
                break;

            case PRE_CLIMB:
                transitionToPreClimb();
                break;

            case ACTIVE_CLIMB:
                transitionToActiveClimb();
                break;

            case STOWED:
                transitionToStowed();
                break;

            case IDLE:
                transitionToIdle();
                break;
        }
    }

    private Command transitionToTraversal() {
        return Commands.parallel(
            // Put down intake
            // Turn on intake
            // Turn off shooter
            // Turn off spindexer
            // Lower climber
            // Unrestrict hood
            // Unrestrict turret
        );
    }

    private Command transitionToShooting() {
        return Commands.parallel(
            // Put down intake
            // Turn on intake
            // Turn on shooter
            // Turn on spindexer
            // Lower climber
            // Unrestrict hood
            // Unrestrict turret
        );
    }

    private Command transitionToTrench() {
        return Commands.parallel(
            // Put down intake
            // Turn on intake
            // Turn off shooter
            // Turn off spindexer
            // Lower climber
            // Restrict hood
            // Unrestrict turret
        );
    }

    private Command transitionToPreClimb() {
        return Commands.parallel(
            // Put down intake
            // Turn on intake
            // Turn on shooter
            // Turn on spindexer
            // Raise climber
            // Unrestrict hood
            // Unrestrict turret
        );
    }

    private Command transitionToActiveClimb() {
        return Commands.parallel(
            // Put down intake
            // Turn off intake
            // Turn on shooter
            // Turn on spindexer
            // Lower climber
            // Unrestrict hood
            // Unrestrict turret
        );
    }

    private Command transitionToStowed() {
        return Commands.parallel(
            // Put up intake
            // Turn off intake
            // Turn off shooter
            // Turn off spindexer
            // Lower climber
            // Unrestrict hood
            // Restrict turret
        );
    }

    private Command transitionToIdle() {
        return Commands.parallel(
            // Put down intake
            // Turn off intake
            // Turn off shooter
            // Turn off spindexer
            // Lower climber
            // Unrestrict hood
            // Unrestrict turret
        );
    }

    public void setStateOverride(SuperState overrideState) {
        override = true;
        transition(overrideState);
    }

    // Something to consider is having a method that will determine the starting state of the robot,
    // Or just have it force it to stow/sit idle

}

