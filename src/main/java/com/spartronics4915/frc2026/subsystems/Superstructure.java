package com.spartronics4915.frc2026.subsystems;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;

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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

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

    // Swerve
    private final SwerveSubsystem swerve;

    // Bla bla bla bla 
    private RobotState currentRobotState;
    private Zone currentZone;
    private Zone previousZone;

    private boolean stateOverride;

    private AutoAimResult result;
    private double lastShotTime;
    private boolean isAutoAiming;

    // Publishing for superstructure logging
    private final StringPublisher currentStatePublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("State").publish();
    private final StringPublisher currentZonePublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("Current Zone").publish();
    private final StringPublisher previousZonePublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getStringTopic("Previous State").publish();

    private final BooleanPublisher isAutoAimingPublisher = NetworkTableInstance.getDefault().getTable("Superstructure").getBooleanTopic("Is Auto Aiming").publish();
    
    private final StructArrayPublisher<Pose3d> successfulShotPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Flywheel/FuelProjectileSuccessfulShot", Pose3d.struct)
        .publish();
    private final StructArrayPublisher<Pose3d> unsuccessfulShotPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Flywheel/FuelProjectileUnsuccessfulShot", Pose3d.struct)
        .publish();

    private final AutoAim autoAim = new AutoAim(
        10,
        0.01,
        new Translation3d(turretTranslation.getX(), turretTranslation.getY(), Units.inchesToMeters(21.443748 + 2.955)),
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
        SwerveSubsystem swerveSubsystem
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

        this.currentZone = getCurrentZone();
        this.previousZone = currentZone;

        this.isAutoAiming = false;

        NetworkTableInstance.getDefault()
            .getStructTopic("target", Translation3d.struct)
            .publish()
            .set(new Translation3d(hubPose.getX(), hubPose.getY(), Units.inchesToMeters(72)));

        SmartDashboard.putData("Auto-Aim Toggle", Commands.runOnce(() -> isAutoAiming = !isAutoAiming));
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
        if (isAutoAiming) result = autoAim.calculateDynamicAim(
            swerve.getRelativePose(), 
            swerve.getFieldVelocity(), 
            new Translation3d(hubPose.getX(), 
            hubPose.getY(), 
            Units.inchesToMeters(72)), 
            (Math.PI * 0.04826 * shooter.getCurrentRPS()) / 2
        ); else result = null;

        if (result != null) {
            if (result.ToF() != -1) hood.setSetpoint(Rotation2d.kCCW_Pi_2.minus(result.pitch()));
            turret.setSetpoint(result.yaw().minus(swerve.getPose().getRotation()).plus(Rotation2d.kCCW_90deg));

            if (Robot.isSimulation() && (Timer.getFPGATimestamp() - lastShotTime) > 0.1) {
                lastShotTime = Timer.getFPGATimestamp();
                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                    swerve.getRobotPose().getTranslation(),
                    turretTranslation.rotateBy(swerve.getRobotPose().getRotation()).rotateBy(result.yaw().unaryMinus()),
                    // new Translation2d(),
                    swerve.getFieldVelocity(),
                    result.yaw(),
                    Inches.of(21.443748 + 2.955),
                    MetersPerSecond.of(10),
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

        //currentZone = getCurrentZone();
        //if (currentZone != previousZone) {
        //    stateOverride = false;
        //    previousZone = currentZone;
        //}

        //if (stateOverride != true) {
        //    switch (currentZone) {
        //        case ALLIANCE_ZONE:
        //            if (currentRobotState == RobotState.SHOOTING) return;
        //            switchState(RobotState.SHOOTING);
        //            break;
//
        //        case TRENCH:
        //            hood.setClamp(HoodClamp.RESTRICTED);
        //            if (currentRobotState == RobotState.TRAVERSAL) return;
        //            switchState(RobotState.TRAVERSAL);
        //            break;
//
        //        case BUMP:
        //            if (currentRobotState == RobotState.CRUISE) return;
        //            switchState(RobotState.CRUISE);
        //            break;
//
        //        case NEUTRAL_ZONE:
        //            if (currentRobotState == RobotState.TRAVERSAL) return;
        //            switchState(RobotState.TRAVERSAL);
        //            break;
//
        //        case OPPONENT_ZONE:
        //            if (currentRobotState == RobotState.CRUISE) return;
        //            switchState(RobotState.CRUISE);
        //            break;
        //    }
        //} 

        //currentStatePublisher.accept(currentRobotState.name());
        //currentZonePublisher.accept(currentZone.name());
        //previousZonePublisher.accept(previousZone.name());

        isAutoAimingPublisher.accept(isAutoAiming);
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
