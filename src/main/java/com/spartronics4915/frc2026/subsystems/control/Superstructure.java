package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;

import java.util.Set;

import com.spartronics4915.frc2026.autos.Autos;
import com.spartronics4915.frc2026.commands.SuperstructureCommands;
import com.spartronics4915.frc2026.commands.SuperstructureCommands.PipelineState;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;
import com.spartronics4915.frc2026.util.control.FieldRegion;
import com.spartronics4915.frc2026.util.control.FieldZoneMap;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Tracks the robot's field zone and automatically schedules superstructure behavior.
 */
public class Superstructure extends SubsystemBase {
    private static final Scope LOG = Telemetry.scope("Control/Superstructure");
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
    private final VisionSubsystem vision;

    private final LaserCan laserCan;

    private final Debouncer ballDebouncer = new Debouncer(noBallsDebounce, DebounceType.kFalling);

    private boolean ballDetectedDebounced = false;

    private final FieldZoneMap<Zone> zoneMap;
    private Zone currentZone = Zone.UNKNOWN;
    private long sampleTimestampUs;

    public Superstructure(
        SwerveSubsystem swerve,
        AutoAimController controller,
        SuperstructureCommands superCommands,
        VisionSubsystem vision
    ) {
        this.swerve = swerve;
        this.controller = controller;
        this.superCommands = superCommands;
        this.vision = vision;
        this.zoneMap = buildZoneMap();

        this.laserCan = new LaserCan(feederLC);

        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (Exception e) {
            System.err.println(
                "Error initializing LaserCan: " + e.getMessage()
            );
        }

        configureTriggers();
    }

    private boolean inTrenchColumn(Translation2d position) {
        return Math.abs(position.minus(hubPose).getY())
            > bumpTrenchDivTransform.getY();
    }

    private Translation2d getClosestHub(Translation2d position) {
        boolean closerToBlueHub =
            position.minus(centerPose).getX() < 0;

        return closerToBlueHub
            ? hubPose
            : centerPose.minus(hubPose).times(2).plus(hubPose);
    }

    private double hubDeltaX(Translation2d position) {
        return position.minus(getClosestHub(position)).getX();
    }

    private FieldZoneMap<Zone> buildZoneMap() {
        FieldZoneMap<Zone> map =
            new FieldZoneMap<>(Zone.OPPONENT_ZONE);

        map.addZone(Zone.TRENCH, pose -> {
            ChassisSpeeds velocity = swerve.getFieldRelativeVelocity();

            Translation2d projected = pose.plus(
                new Translation2d(
                    velocity.vxMetersPerSecond,
                    velocity.vyMetersPerSecond
                ).times(TRENCH_LOOKAHEAD_SEC)
            );

            double hubPosition = hubDeltaX(pose);
            double projectedHubPosition = hubDeltaX(projected);

            boolean atPose =
                inTrenchColumn(pose)
                && Math.abs(hubPosition) < trenchLength.in(Meters) / 2.0;

            boolean atFuturePose =
                inTrenchColumn(projected)
                && Math.abs(projectedHubPosition)
                    < trenchLength.in(Meters) / 2.0;

            boolean projectedValid =
                (hubPosition * projectedHubPosition) <= 0.0
                && (inTrenchColumn(pose) || inTrenchColumn(projected));

            return atPose || atFuturePose || projectedValid;
        });

        map.addZone(
            Zone.BUMP,
            position ->
                !inTrenchColumn(position)
                && Math.abs(hubDeltaX(position))
                    < bumpLength.in(Meters) / 2.0
        );

        map.addZone(
            Zone.TOWER,
            FieldRegion.rectangle(
                (towerPose.getX() / 2) - towerXTransform,
                (towerPose.getX() / 2) + towerXTransform,
                towerPose.getY() - towerYTransform,
                towerPose.getY() + towerYTransform
            )
        );

        map.addZone(Zone.NEUTRAL_ZONE, position -> {
            double dx = hubDeltaX(position);
            boolean closerToBlueHub =
                position.minus(centerPose).getX() < 0;

            return closerToBlueHub ? dx > 0 : dx < 0;
        });

        map.addZone(
            Zone.ALLIANCE_ZONE,
            position -> position.minus(centerPose).getX() < 0
        );

        return map;
    }

    @Override
    public void periodic() {
        Pose2d pose = swerve.getRelativePose();

        Translation2d turretTranslation =
            pose.getTranslation().plus(
                turretTranslation2D.rotateBy(pose.getRotation())
            );

        Zone newZone = zoneMap.evaluate(turretTranslation);

        if (newZone != currentZone) {
            currentZone = newZone;
        }

        ballDetectedDebounced =
            ballDebouncer.calculate(ballDetect());

        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("CurrentZone", currentZone);
        LOG.critical.log("BallDetected", ballDetectedDebounced);
    }

    private boolean ballDetect() {
        LaserCan.Measurement measurement =
            laserCan.getMeasurement();

        if (
            measurement == null
            || measurement.status
                != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        ) {
            return true;
        }

        return measurement.distance_mm < detectDistance;
    }

    public boolean isBallDetectedDebounced() {
        return ballDetectedDebounced;
    }

    private void configureTriggers() {
        new Trigger(() -> currentZone == Zone.ALLIANCE_ZONE).onTrue(
            superCommands
                .shooting()
                .withName("Auto: Shooting Zone")
                .ignoringDisable(true)
        );

        new Trigger(() -> currentZone == Zone.TRENCH).onTrue(
            superCommands
                .trench()
                .withName("Auto: Trench Traversal")
                .ignoringDisable(true)
        );

        new Trigger(() -> currentZone == Zone.NEUTRAL_ZONE).onTrue(
            superCommands
                .traversal()
                .withName("Auto: Neutral Traversal")
                .ignoringDisable(true)
        );

        new Trigger(() ->
                currentZone == Zone.BUMP
                || currentZone == Zone.OPPONENT_ZONE
        ).onTrue(
            superCommands
                .cruise()
                .withName("Auto: Cruise Zone")
                .ignoringDisable(true)
        );

        new Trigger(controller::isReadyToShoot)
            .onTrue(
                superCommands.setPipelineState(PipelineState.ON)
            )
            .onFalse(
                superCommands.setPipelineState(PipelineState.OFF)
            );

        RobotModeTriggers.teleop().onTrue(
            Commands.runOnce(() -> controller.setShootingState(false))
        );
    }

    public Command getReturnToZoneCommand() {
        return Commands.defer(() -> {
            return switch (currentZone) {
                case ALLIANCE_ZONE -> superCommands.shooting();
                case TRENCH -> superCommands.trench();
                case NEUTRAL_ZONE -> superCommands.traversal();
                case BUMP, OPPONENT_ZONE -> superCommands.cruise();
                default -> superCommands.idle();
            };
        }, Set.of()).withName("Restore Zone State");
    }

    /**
     * Jostles the pivot back and forth, then leaves it in READY (down), stopping 0.5s before `duration` finishes.
     */
    public Command getJostleCommand(double duration) {
        if (duration <= 0.5) return Commands.sequence(
            superCommands.conditionalPivotReady(),
            Autos.wait(duration)
        );
        
        return Commands.sequence(
            Commands.deadline(
                Autos.wait(duration - 0.5),
                Commands.sequence(
                    superCommands.conditionalPivotSafe(),
                    Commands.waitSeconds(0.5 / PIVOT_JOSTLE_FREQUENCY),
                    superCommands.conditionalPivotReady(),
                    Commands.waitSeconds(0.5 / PIVOT_JOSTLE_FREQUENCY)
                ).repeatedly()
            ),
            superCommands.conditionalPivotReady(),
            Autos.wait(0.5)
        );
    }
}
