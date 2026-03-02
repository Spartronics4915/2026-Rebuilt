package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;

import static edu.wpi.first.units.Units.Meters;

import com.spartronics4915.frc2026.commands.SuperstructureCommands;
import com.spartronics4915.frc2026.commands.SuperstructureCommands.PipelineState;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2026.util.FieldRegion;
import com.spartronics4915.frc2026.util.FieldZoneMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Tracks the robot's position on the field and automatically schedules 
 * SuperstructureCommands based on the current zone using Triggers.
 *
 * <p>Set {@link #testingMode} to {@code true} to disable all automatic zone
 * triggers and pipeline management. Zone tracking and NetworkTables publishing
 * will continue to run regardless, so field position can still be observed.
 * Set to {@code false} to restore full autonomous zone behavior.
 */
public class Superstructure extends SubsystemBase {

    /**
     * When true, zone triggers and automatic pipeline management are disabled.
     * The robot will still track and publish the current zone to NetworkTables,
     * but no SuperstructureCommands will be scheduled automatically.
     * Set to false to restore full autonomous zone behavior.
     */
    private static boolean testingMode = true;
    
    public enum Zone {
        ALLIANCE_ZONE,
        TRENCH,
        BUMP,
        NEUTRAL_ZONE,
        OPPONENT_ZONE,
        UNKNOWN
    }

    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final AutoAimController controller;
    private final SuperstructureCommands commands;

    private final FieldZoneMap<Zone> zoneMap;
    private Zone currentZone = Zone.UNKNOWN;

    private final StringPublisher zonePublisher = 
        NetworkTableInstance.getDefault().getStringTopic("superstructure/Current Zone").publish();

    public Superstructure(
        SwerveSubsystem swerve, 
        VisionSubsystem vision, 
        AutoAimController controller,
        SuperstructureCommands commands
    ) {
        this.swerve = swerve;
        this.vision = vision;
        this.controller = controller;
        this.commands = commands;

        this.zoneMap = buildZoneMap();
        
        configureZoneTriggers();

        SmartDashboard.putData("Toggle Testing", toggleTestingMode());
    }

    private FieldZoneMap<Zone> buildZoneMap() {
        FieldZoneMap<Zone> map = new FieldZoneMap<>(Zone.OPPONENT_ZONE);

        FieldRegion isNearHub = (pos) -> {
            boolean inTrenchColumn = Math.abs(pos.minus(hubPose).getY()) > bumpTrenchDivTransform.getY();
            boolean closerToBlueHub = pos.minus(centerPose).getX() < 0;
            Translation2d nearestHub = closerToBlueHub ? hubPose : centerPose.minus(hubPose).times(2).plus(hubPose);
            
            double hubDeltaX = pos.minus(nearestHub).getX();
            double halfLength = (inTrenchColumn ? trenchLength : bumpLength).in(Meters) / 2.0;
            return Math.abs(hubDeltaX) < halfLength;
        };

        map.addZone(Zone.TRENCH, pos -> isNearHub.contains(pos) && 
            Math.abs(pos.minus(hubPose).getY()) > bumpTrenchDivTransform.getY());

        map.addZone(Zone.BUMP, pos -> isNearHub.contains(pos) && 
            Math.abs(pos.minus(hubPose).getY()) <= bumpTrenchDivTransform.getY());

        map.addZone(Zone.NEUTRAL_ZONE, pos -> {
            boolean closerToBlueHub = pos.minus(centerPose).getX() < 0;
            Translation2d nearestHub = closerToBlueHub ? hubPose : centerPose.minus(hubPose).times(2).plus(hubPose);
            double hubDeltaX = pos.minus(nearestHub).getX();
            return closerToBlueHub ? hubDeltaX > 0 : hubDeltaX < 0; 
        });

        map.addZone(Zone.ALLIANCE_ZONE, pos -> pos.minus(centerPose).getX() < 0);

        return map;
    }

    @Override
    public void periodic() {
        if (vision.hasValidPose()) {
            zonePublisher.accept(currentZone.name());
        } //else {
            //return;
        //}

        Translation2d turretTranslation = swerve.getRelativePose().getTranslation()
            .plus(TURRET_TRANSLATION.rotateBy(swerve.getPose().getRotation()));

        Zone newZone = zoneMap.evaluate(turretTranslation);

        if (controller.hasValidResult()) {
            CommandScheduler.getInstance().schedule(commands.setPipelineState(PipelineState.ON));
        } else {
            CommandScheduler.getInstance().schedule(commands.setPipelineState(PipelineState.OFF));
        }
        
        if (newZone != currentZone) {
            currentZone = newZone;
            zonePublisher.accept(currentZone.name());
        }
    }

    private void configureZoneTriggers() {
        if (testingMode) return;

        Trigger inAllianceZone = new Trigger(() -> currentZone == Zone.ALLIANCE_ZONE);
        Trigger inTrench = new Trigger(() -> currentZone == Zone.TRENCH);
        Trigger inNeutralZone = new Trigger(() -> currentZone == Zone.NEUTRAL_ZONE);
        Trigger inCruiseZones = new Trigger(() -> currentZone == Zone.BUMP || currentZone == Zone.OPPONENT_ZONE);

        inAllianceZone.onTrue(commands.shooting().withName("Auto: Shooting Zone"));
        inTrench.onTrue(commands.trenchTraversal().withName("Auto: Trench Traversal"));
        inNeutralZone.onTrue(commands.traversal().withName("Auto: Neutral Traversal"));
        inCruiseZones.onTrue(commands.cruise().withName("Auto: Cruise Zone"));
    }

    /**
     * Called when a manual driver override is released to snap the robot back 
     * to whatever state it should be in based on its field position.
     */
    public Command getReturnToZoneCommand() {
        return Commands.defer(() -> {
            switch (currentZone) {
                case ALLIANCE_ZONE: return commands.shooting();
                case TRENCH: return commands.trenchTraversal();
                case NEUTRAL_ZONE: return commands.traversal();
                case BUMP:          
                case OPPONENT_ZONE: return commands.cruise();
                default: return commands.idle();
            }
        }, java.util.Set.of()).withName("Restore Zone State");
    }

    public Command toggleTestingMode() {
        return Commands.runOnce(() -> testingMode = !testingMode).ignoringDisable(true);
    }
}