package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import com.spartronics4915.frc2026.commands.DriveCommand;
import com.spartronics4915.frc2026.commands.DriveCommand.SpeedLimitMode;
import com.spartronics4915.frc2026.commands.SuperstructureCommands;
import com.spartronics4915.frc2026.commands.SuperstructureCommands.PipelineState;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2026.util.control.FieldRegion;
import com.spartronics4915.frc2026.util.control.FieldZoneMap;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Tracks the robot's position on the field and automatically schedules
 * SuperstructureCommands based on the current zone using Triggers
 */
public class Superstructure extends SubsystemBase {

    public enum Zone {
        ALLIANCE_ZONE,
        TRENCH,
        BUMP,
        TOWER,
        NEUTRAL_ZONE,
        OPPONENT_ZONE,
        UNKNOWN
    }

    private final SwerveSubsystem swerve;
    private final AutoAimController controller;
    private final SuperstructureCommands superCommands;
    private final DriveCommand driveCommand;
    private final VisionSubsystem vision;

    private final LaserCan laserCan;

    private final Debouncer ballDebouncer = new Debouncer(noBallsDebounce, DebounceType.kFalling);
    private boolean ballDetectedDebounced = false;

    private final FieldZoneMap<Zone> zoneMap;
    private Zone currentZone = Zone.UNKNOWN;

    private final StringPublisher zonePublisher =
        NetworkTableInstance.getDefault().getStringTopic("superstructure/Current Zone").publish();
    private final BooleanPublisher ballDetectedPublisher =
        NetworkTableInstance.getDefault().getBooleanTopic("superstructure/Ball Detect").publish();

    public Superstructure(
        SwerveSubsystem swerve,
        AutoAimController controller,
        SuperstructureCommands superCommands,
        DriveCommand driveCommand,
        VisionSubsystem vision
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.superCommands = superCommands;
        this.driveCommand = driveCommand;
        this.vision = vision;
        this.zoneMap = buildZoneMap();

        this.laserCan = new LaserCan(feederLC);
        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (Exception e) {
            System.err.println("Error initializing LaserCan: " + e.getMessage());
        }

        configureTriggers();
    }

    // Zone map -----------------------------------------------------

    private boolean inTrenchColumn(Translation2d pos) {
        return Math.abs(pos.minus(hubPose).getY()) > bumpTrenchDivTransform.getY();
    }

    private Translation2d getClosestHub(Translation2d pos) {
        boolean closerToBlueHub = pos.minus(centerPose).getX() < 0;
        return closerToBlueHub ? hubPose : centerPose.minus(hubPose).times(2).plus(hubPose);
    }

    private double hubDeltaX(Translation2d pos) {
        return pos.minus(getClosestHub(pos)).getX();
    }

    private FieldZoneMap<Zone> buildZoneMap() {
        FieldZoneMap<Zone> map = new FieldZoneMap<>(Zone.OPPONENT_ZONE);

        // Checks both the current position and a velocity projected position so the
        // hood lowers before the robot physically enters the trench
        map.addZone(Zone.TRENCH, pose -> {
            ChassisSpeeds vel = swerve.getFieldRelativeVelocity();
            Translation2d projected = pose.plus(
                new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond)
                    .times(TRENCH_LOOKAHEAD_SEC)
            );

            double hubPos = hubDeltaX(pose);
            double hubProjected = hubDeltaX(projected);

            boolean atPose = inTrenchColumn(pose) && Math.abs(hubPos) < trenchLength.in(Meters) / 2.0;
            boolean atFuturePose = inTrenchColumn(projected) && Math.abs(hubProjected) < trenchLength.in(Meters) / 2.0;

            boolean projectedValid = (hubPos * hubProjected) <= 0.0 && (inTrenchColumn(pose) || inTrenchColumn(projected));

            return atPose || atFuturePose|| projectedValid;
        });

        map.addZone(Zone.BUMP, pos ->
            !inTrenchColumn(pos) && Math.abs(hubDeltaX(pos)) < bumpLength.in(Meters) / 2.0);

        map.addZone(Zone.TOWER, 
            FieldRegion.rectangle(
                (towerPose.getX() / 2) - towerXTransform, 
                (towerPose.getX() / 2) + towerXTransform, 
                towerPose.getY() - towerYTransform, 
                towerPose.getY() + towerYTransform
            )
        );

        map.addZone(Zone.NEUTRAL_ZONE, pos -> {
            double dx = hubDeltaX(pos);
            boolean closerToBlueHub = pos.minus(centerPose).getX() < 0;
            return closerToBlueHub ? dx > 0 : dx < 0;
        });

        map.addZone(Zone.ALLIANCE_ZONE, pos -> pos.minus(centerPose).getX() < 0);

        return map;
    }

    // Periodic -----------------------------------------------------

    @Override
    public void periodic() {
        Translation2d turretTranslation = swerve.getSmoothedRelativePose().getTranslation()
            .plus(turretTranslation2D.rotateBy(swerve.getSmoothedRelativePose().getRotation()));

        Zone newZone = zoneMap.evaluate(turretTranslation);

        if (newZone != currentZone) {
            currentZone = newZone;
            zonePublisher.accept(currentZone.name());
        }

        // Apply a speed limit while ready-to-shoot to help the turret track cleanly.
        if (controller.isTryingToShoot()) {
            if (controller.isPassTarget()) {
                driveCommand.setSpeedLimit(SpeedLimitMode.FERRY);
            } else {
                driveCommand.setSpeedLimit(SpeedLimitMode.HUB);
            }
        } else {
            driveCommand.setSpeedLimit(SpeedLimitMode.OFF);
        }

        Pose2d pose = swerve.getPose();
        
        if (pose != null) {
            if (pose.getX() < 0) {
                swerve.resetPose(vision.getVisionPose());
            } else if (pose.getY() < 0.0 || pose.getY() > 8.1) {
                swerve.resetPose(vision.getVisionPose());
            }
        }

        ballDetectedDebounced = ballDebouncer.calculate(ballDetect());
        ballDetectedPublisher.set(ballDetectedDebounced);
    }

    private boolean ballDetect(){
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return true;
        } else{
            return measurement.distance_mm < detectDistance;
        }
    }

    public boolean isBallDetectedDebounced() {
        return ballDetectedDebounced;
    }

    // Triggers -----------------------------------------------------

    private void configureTriggers() {
        // Zone triggers
        new Trigger(() -> currentZone == Zone.ALLIANCE_ZONE)
            .onTrue(superCommands.shooting().withName("Auto: Shooting Zone"));
        new Trigger(() -> currentZone == Zone.TRENCH)
            .onTrue(superCommands.trench().withName("Auto: Trench Traversal"));
        new Trigger(() -> currentZone == Zone.NEUTRAL_ZONE)
            .onTrue(superCommands.traversal().withName("Auto: Neutral Traversal"));
        new Trigger(() -> currentZone == Zone.BUMP || currentZone == Zone.OPPONENT_ZONE)
            .onTrue(superCommands.cruise().withName("Auto: Cruise Zone"));

        // Pipeline triggers, falling edge debounced so the pipeline turns on instantly
        // but won't turn off until isReadyToShoot has been false for the full duration.
        Trigger pipelineOn = new Trigger(controller::isReadyToShoot)
            .onTrue(superCommands.setPipelineState(PipelineState.ON))
            .onFalse(superCommands.setPipelineState(PipelineState.OFF));

        // In auto, jostle the pivot while the pipeline is active
        pipelineOn
            .and(DriverStation::isAutonomous)
            .whileTrue(Commands.sequence(
                superCommands.conditionalPivotSafe(),
                Commands.waitSeconds(0.5 / PIVOT_JOSTLE_FREQUENCY),
                superCommands.conditionalPivotReady(),
                Commands.waitSeconds(0.5 / PIVOT_JOSTLE_FREQUENCY)
            ).repeatedly())
            .whileFalse(superCommands.conditionalPivotReady());
        
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
            controller.setShootingState(false);
        }));
    }

    /**
     * Called when a manual driver override is released to snap the robot back
     * to whatever state it should be in based on its field position.
     */
    public Command getReturnToZoneCommand() {
        return Commands.defer(() -> {
            switch (currentZone) {
                case ALLIANCE_ZONE: return superCommands.shooting();
                case TRENCH: return superCommands.trench();
                case NEUTRAL_ZONE: return superCommands.traversal();
                case BUMP:
                case OPPONENT_ZONE: return superCommands.cruise();
                default: return superCommands.idle();
            }
        }, Set.of()).withName("Restore Zone State");
    }

}
