package com.spartronics4915.frc2026.subsystems;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;

import com.spartronics4915.frc2026.Robot;
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
import com.spartronics4915.frc2026.util.AutoAim;
import com.spartronics4915.frc2026.util.AutoAim.AutoAimResult;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

/**
 * Coordinates all robot mechanisms (hood, turret, feeder, indexer, shooter,
 * climber, intake, pivot) through a zone-based state machine.
 *
 * <p>Manages automatic state transitions based on field position, dynamic
 * auto-aim calculations via {@link AutoAim}, and simulation projectile
 * visualization. State transitions are driven by {@link Zone} changes or
 * explicit overrides via {@link #setStateOverride}.
 */
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

    // Other subsystems
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    // State machine tracking
    private RobotState currentRobotState;
    private Zone currentZone;
    private Zone previousZone;

    private boolean stateOverride;

    private AutoAimResult result;
    private double lastShotTime;
    private boolean isAutoAiming;

    // Publishing for superstructure logging    
    private static final NetworkTable superTable = NetworkTableInstance.getDefault().getTable("superstructure");

    private final StringPublisher currentStatePublisher = superTable.getStringTopic("State").publish();
    private final StringPublisher currentZonePublisher = superTable.getStringTopic("Current Zone").publish();
    private final StringPublisher previousZonePublisher = superTable.getStringTopic("Previous State").publish();

    private final BooleanPublisher isAutoAimingPublisher = superTable.getBooleanTopic("Is Auto Aiming").publish();
    
    private final StructArrayPublisher<Pose3d> successfulShotPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Flywheel/FuelProjectileSuccessfulShot", Pose3d.struct)
        .publish();
    private final StructArrayPublisher<Pose3d> unsuccessfulShotPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Flywheel/FuelProjectileUnsuccessfulShot", Pose3d.struct)
        .publish();

    private final AutoAim autoAim = new AutoAim(
        10,
        0.01,
        new Translation3d(
            TURRET_TRANSLATION.getX(), 
            TURRET_TRANSLATION.getY(), 
            Units.inchesToMeters(21.443748 + 2.955)
        ),
        Rotation2d.fromDegrees(50),
        Rotation2d.fromDegrees(90)
    );
    
    public Superstructure(
        HoodSubsystem hoodSubsystem,
        TurretSubsystem turretSubsystem,
        FeederSubsystem feederSubsystem,
        IndexerSubsystem indexerSubsystem,
        ShooterSubsystem shooterSubsystem,
        ClimberSubsystem climberSubsystem,
        IntakeSubsystem intakeSubsystem,
        PivotSubsystem pivotSubsystem,
        SwerveSubsystem swerveSubsystem,
        VisionSubsystem visionSubsystem
    ) {
        this.hood = hoodSubsystem;
        this.turret = turretSubsystem;
        this.feeder = feederSubsystem;
        this.indexer = indexerSubsystem;
        this.shooter = shooterSubsystem;
        this.climber = climberSubsystem;
        this.intake = intakeSubsystem;
        this.pivot = pivotSubsystem;
        this.swerve = swerveSubsystem;
        this.vision = visionSubsystem;

        this.currentZone = vision.hasValidPose()
            ? getCurrentZone(swerve.getRelativePose().getTranslation().plus(
                 TURRET_TRANSLATION.rotateBy(swerve.getPose().getRotation())
            ))
            : Zone.OPPONENT_ZONE;

        this.previousZone = currentZone;

        this.isAutoAiming = false;

        this.currentRobotState = RobotState.TESTING;
        this.stateOverride = true;

        NetworkTableInstance.getDefault()
            .getStructTopic("target", Translation3d.struct)
            .publish()
            .set(new Translation3d(hubPose.getX(), hubPose.getY(), Units.inchesToMeters(72)));

        SmartDashboard.putData("Auto-Aim Toggle", Commands.runOnce(() -> isAutoAiming = !isAutoAiming).ignoringDisable(true));

        SmartDashboard.putData("Traversal", Commands.deferredProxy(this::transToTraversal));
        SmartDashboard.putData("Cruise", Commands.deferredProxy(this::transToCruise));
        SmartDashboard.putData("Shooting", Commands.deferredProxy(this::transToShooting));
        SmartDashboard.putData("Climb", Commands.deferredProxy(this::transToClimb));
        SmartDashboard.putData("Idle", Commands.deferredProxy(this::transToIdle));
        SmartDashboard.putData("Stowed", Commands.deferredProxy(this::transToStowed));

        SmartDashboard.putData(
            "Reset Dynamics",
            Commands.parallel(
                turret.setSetpointCommand(Rotation2d.fromDegrees(0)),
                hood.setSetpointCommand(Rotation2d.fromDegrees(0))
            )
        );
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
        currentZone = getCurrentZone(swerve.getRelativePose().getTranslation().plus(
            TURRET_TRANSLATION.rotateBy(swerve.getPose().getRotation())
        ));

        if (currentZone != previousZone) {
            stateOverride = false;
            previousZone = currentZone;
        }
        
        if (stateOverride != true && currentRobotState != RobotState.TESTING) {
            Command command = null;
            switch (currentZone) {
                case ALLIANCE_ZONE:
                    if (currentRobotState != RobotState.SHOOTING) {
                        command = switchState(RobotState.SHOOTING);
                    }
                    break;
                
                case TRENCH:
                    hood.setClamp(HoodClamp.RESTRICTED);
                    if (currentRobotState != RobotState.TRAVERSAL) {
                        command = switchState(RobotState.TRAVERSAL);
                    }
                    break;
                
                case BUMP:
                    if (currentRobotState != RobotState.CRUISE) {
                        command = switchState(RobotState.CRUISE);
                    }
                    break;
                
                case NEUTRAL_ZONE:
                    if (currentRobotState != RobotState.TRAVERSAL) {
                        command = switchState(RobotState.TRAVERSAL);
                    }
                    break;
                
                case OPPONENT_ZONE:
                    if (currentRobotState != RobotState.CRUISE) {
                        command = switchState(RobotState.CRUISE);
                    }
                    break;
            }
        
            if (command != null) {
                CommandScheduler.getInstance().schedule(command);
            }
        }

        if (isAutoAiming) {
            result = autoAim.calculateDynamicAim(
                swerve.getRelativePose(), 
                swerve.getRelativeFieldVelocity(), 
                new Translation3d(
                    hubPose.getX(), 
                    hubPose.getY(), 
                    Units.inchesToMeters(72)
                ), 
                InchesPerSecond.of(shooter.getCurrentRPS() * Math.PI * 1.92).in(MetersPerSecond) * (1 - percentLoss)
            );
        } else {
            result = null;
        } 

        if (result != null) {
            if (result.ToF() != -1) hood.setSetpoint(Rotation2d.kCCW_Pi_2.minus(result.pitch()));
            turret.setSetpoint(result.yaw().minus(swerve.getPose().getRotation()).minus(Rotation2d.kCCW_90deg));

            if (Robot.isSimulation() && (Timer.getFPGATimestamp() - lastShotTime) > 0.1 && result.ToF() != -1) {
                lastShotTime = Timer.getFPGATimestamp();
                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                    swerve.getRobotPose().getTranslation(),
                    TURRET_TRANSLATION.rotateBy(swerve.getRobotPose().getRotation()).rotateBy(result.yaw().unaryMinus()),
                    // new Translation2d(),
                    swerve.getFieldVelocity(),
                    result.yaw(),
                    Inches.of(21.443748 + 2.955),
                    InchesPerSecond.of(shooter.getCurrentRPS() * Math.PI * 1.92),
                    Degrees.of(result.pitch().getDegrees())
                );

                fuelOnFly
                    // Set the target center to the Rebuilt Hub of the current alliance
                    .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(hubPose.getX(), hubPose.getY(), Units.inchesToMeters(62))))
                    .withTargetTolerance(new Translation3d(0.67, 0.67, 0.3));
                
                fuelOnFly
                    .withProjectileTrajectoryDisplayCallBack(
                        (pose3ds) -> {successfulShotPublisher.set(pose3ds.toArray(Pose3d[]::new)); unsuccessfulShotPublisher.set(new Pose3d[0]);},
                        (pose3ds) -> {unsuccessfulShotPublisher.set(pose3ds.toArray(Pose3d[]::new)); successfulShotPublisher.set(new Pose3d[0]);}
                    );
                fuelOnFly.disableBecomesGamePieceOnFieldAfterTouchGround();

                SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
            }
        }

        currentStatePublisher.accept(currentRobotState.name());
        currentZonePublisher.accept(currentZone.name());
        previousZonePublisher.accept(previousZone.name());

        isAutoAimingPublisher.accept(isAutoAiming);
    }

    public void setStateOverride(RobotState overrideState) {
        stateOverride = true;
        CommandScheduler.getInstance().schedule(switchState(overrideState));
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

    private Command switchState(RobotState desiredState) {
        currentRobotState = desiredState;
        switch (desiredState) {
            case TRAVERSAL:
                return transToTraversal();

            case CRUISE:
                return transToCruise();

            case SHOOTING:
                return transToShooting();

            case CLIMB:
                return transToClimb();

            case IDLE:
                return transToIdle();

            case STOWED:
                return transToStowed();

            default:
                return Commands.none();
        }
    }

    private Zone getCurrentZone(Translation2d position) {
        // Checks which zone it is vertically (trench or bump), it considers the hub to be part of the bump
        boolean trenchZone = Math.abs(position.minus(hubPose).getY()) > bumpTrenchDivTransform.getY();

        // Mirror across center to get the red hub if required
        boolean closeHub = position.minus(centerPose).getX() < 0;
        Translation2d closestHub = closeHub ? hubPose : centerPose.minus(hubPose).times(2).plus(hubPose);

        double hubDist = position.minus(closestHub).getX();
        if (Math.abs(hubDist) < (trenchZone ? trenchLength : bumpLength).in(Meters) / 2) {
            return trenchZone ? Zone.TRENCH : Zone.BUMP;
        }

        // Past the first bump/trench before the second bump/trench
        if (closeHub ? hubDist > 0 : hubDist < 0) {
            return Zone.NEUTRAL_ZONE;
        } else {
            return closeHub ? Zone.ALLIANCE_ZONE : Zone.OPPONENT_ZONE;
        }
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
