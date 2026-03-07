package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import com.spartronics4915.frc2026.commands.SuperstructureCommands;
import com.spartronics4915.frc2026.commands.SuperstructureCommands.PipelineState;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.control.FieldZoneMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

    // Debounce for pipeline state switching: require the controller's "ready to shoot"
    // signal to be stable for this many seconds before actually scheduling the change.
    private static final double PIPELINE_DEBOUNCE_SEC = 0.2;
    private boolean debouncedPipelineOn = false;
    private final Trigger pipelineTrigger;

    public Superstructure(
        SwerveSubsystem swerve, 
        AutoAimController controller,
        SuperstructureCommands commands
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.commands = commands;
        this.zoneMap = buildZoneMap();
        // Create a debounced Trigger from the controller's ready-to-shoot signal.
        this.pipelineTrigger = new Trigger(this.controller::isReadyToShoot).debounce(PIPELINE_DEBOUNCE_SEC);
        this.debouncedPipelineOn = this.pipelineTrigger.getAsBoolean();

        configureZoneTriggers();
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

        map.addZone(Zone.TRENCH, pos ->
            inTrenchColumn(pos) && Math.abs(hubDeltaX(pos)) < trenchLength.in(Meters) / 2.0);

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

        // Use a debounced Trigger and read its current debounced boolean value.
        boolean debounced = pipelineTrigger.getAsBoolean();
        if (debounced != debouncedPipelineOn) {
            debouncedPipelineOn = debounced;
            CommandScheduler.getInstance().schedule(
                commands.setPipelineState(debouncedPipelineOn ? PipelineState.ON : PipelineState.OFF)
            );
        }
        
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
                case TRENCH:        return commands.trench();
                case NEUTRAL_ZONE:  return commands.traversal();
                case BUMP:          
                case OPPONENT_ZONE: return commands.cruise();
                default:            return commands.idle();
            }
        }, Set.of()).withName("Restore Zone State");
    }
}
