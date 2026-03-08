package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import com.spartronics4915.frc2026.commands.SuperstructureCommands;
import com.spartronics4915.frc2026.commands.SuperstructureCommands.PipelineState;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.control.FieldZoneMap;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Tracks the robot's position on the field and automatically schedules 
 * SuperstructureCommands based on the current zone using Triggers.
 */
public class Superstructure extends SubsystemBase {
    
    public enum Zone {
        ALLIANCE_ZONE,
        TRENCH,
        BUMP,
        NEUTRAL_ZONE,
        OPPONENT_ZONE,
        UNKNOWN
    }

    private final SwerveSubsystem swerve;
    private final AutoAimController controller;
    private final SuperstructureCommands commands;

    private final FieldZoneMap<Zone> zoneMap;
    private Zone currentZone = Zone.UNKNOWN;

    private final StringPublisher zonePublisher = 
        NetworkTableInstance.getDefault().getStringTopic("superstructure/Current Zone").publish();

    public Superstructure(
        SwerveSubsystem swerve, 
        AutoAimController controller,
        SuperstructureCommands commands
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.commands = commands;
        this.zoneMap = buildZoneMap();

        configureZoneTriggers();

        // Debounce both edges so the pipeline doesn't toggle rapidly if
        // isReadyToShoot flickers around the threshold.
        new Trigger(controller::isReadyToShoot)
            .debounce(PIPELINE_RATE_LIMIT_SEC, DebounceType.kFalling)
            .onTrue(commands.setPipelineState(PipelineState.ON))
            .onFalse(commands.setPipelineState(PipelineState.OFF));
    }

    private boolean inTrenchColumn(Translation2d pos) {
        return Math.abs(pos.minus(hubPose).getY()) > bumpTrenchDivTransform.getY();
    }

    private Translation2d nearestHub(Translation2d pos) {
        boolean closerToBlueHub = pos.minus(centerPose).getX() < 0;
        return closerToBlueHub ? hubPose : centerPose.minus(hubPose).times(2).plus(hubPose);
    }

    private double hubDeltaX(Translation2d pos) {
        return pos.minus(nearestHub(pos)).getX();
    }

    private FieldZoneMap<Zone> buildZoneMap() {
        FieldZoneMap<Zone> map = new FieldZoneMap<>(Zone.OPPONENT_ZONE);

        // Checks both the current position and a velocity-projected position so the
        // hood lowers before the robot physically enters the trench. All other zones
        // are position only.
        map.addZone(Zone.TRENCH, pos -> {
            ChassisSpeeds vel = swerve.getFieldRelativeVelocity();
            Translation2d projected = pos.plus(
                new Translation2d(vel.vxMetersPerSecond, vel.vyMetersPerSecond)
                    .times(TRENCH_LOOKAHEAD_SEC));
            return inTrenchColumn(pos) && Math.abs(hubDeltaX(pos)) < trenchLength.in(Meters) / 2.0
                || inTrenchColumn(projected) && Math.abs(hubDeltaX(projected)) < trenchLength.in(Meters) / 2.0;
        });

        map.addZone(Zone.BUMP, pos ->
            !inTrenchColumn(pos) && Math.abs(hubDeltaX(pos)) < bumpLength.in(Meters) / 2.0);

        map.addZone(Zone.NEUTRAL_ZONE, pos -> {
            double dx = hubDeltaX(pos);
            boolean closerToBlueHub = pos.minus(centerPose).getX() < 0;
            return closerToBlueHub ? dx > 0 : dx < 0;
        });

        map.addZone(Zone.ALLIANCE_ZONE, pos -> pos.minus(centerPose).getX() < 0);

        return map;
    }

    @Override
    public void periodic() {
        Translation2d turretTranslation = swerve.getRelativePose().getTranslation()
            .plus(TURRET_TRANSLATION.rotateBy(swerve.getPose().getRotation()));

        Zone newZone = zoneMap.evaluate(turretTranslation);

        if (newZone != currentZone) {
            currentZone = newZone;
            zonePublisher.accept(currentZone.name());
        }
    }

    private void configureZoneTriggers() {
        new Trigger(() -> currentZone == Zone.ALLIANCE_ZONE)
            .onTrue(commands.shooting().withName("Auto: Shooting Zone"));
        new Trigger(() -> currentZone == Zone.TRENCH)
            .onTrue(commands.trench().withName("Auto: Trench Traversal"));
        new Trigger(() -> currentZone == Zone.NEUTRAL_ZONE)
            .onTrue(commands.traversal().withName("Auto: Neutral Traversal"));
        new Trigger(() -> currentZone == Zone.BUMP || currentZone == Zone.OPPONENT_ZONE)
            .onTrue(commands.cruise().withName("Auto: Cruise Zone"));
    }
    
    /**
     * Called when a manual driver override is released to snap the robot back 
     * to whatever state it should be in based on its field position.
     */
    public Command getReturnToZoneCommand() {
        return Commands.defer(() -> {
            switch (currentZone) {
                case ALLIANCE_ZONE: return commands.shooting();
                case TRENCH: return commands.trench();
                case NEUTRAL_ZONE: return commands.traversal();
                case BUMP:          
                case OPPONENT_ZONE: return commands.cruise();
                default: return commands.idle();
            }
        }, Set.of()).withName("Restore Zone State");
    }
}