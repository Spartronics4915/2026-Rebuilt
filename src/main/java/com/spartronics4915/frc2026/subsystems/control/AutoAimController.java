package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static com.spartronics4915.frc2026.Constants.AutoAimConstants.*;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem.ShooterClamp;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.control.AutoAim;
import com.spartronics4915.frc2026.util.control.AutoAim.AutoAimResult;
import com.spartronics4915.frc2026.util.control.TurretController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

/**
 * Continuously calculates setpoints and applies them to the hood and turret.
 * Simulation projectile visualization is handled here as well.
 *
 * <p>Auto-aim can be toggled at any time, including while the robot is disabled,
 * via {@link #aimToggle()}. When disabled, the hood and turret retain their last
 * commanded setpoints.
 *
 * <p>The calculation pipeline each loop:
 * <ol>
 *   <li>Ask {@link AutoAim} for yaw, pitch, and time-of-flight.
 *   <li>Push yaw to the turret via {@link TurretController} and pitch to the hood.
 *   <li>If in simulation, spawn a {@link RebuiltFuelOnFly} projectile for visualization.
 * </ol>
 */
public class AutoAimController extends SubsystemBase {

    private final HoodSubsystem hood;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;

    private final AutoAim autoAim = new AutoAim(
        30,
        0.01,
        TURRET_TRANSLATION_3D,
        Rotation2d.fromDegrees(50),
        Rotation2d.fromDegrees(90),
        RPSToMPS(MAX_SHOOTER_RPS),
        0.02
    );

    private final TurretController turretController;

    private boolean isAimEnabled = false;
    private boolean isShootingEnabled = false;
    private AutoAimResult lastResult = null;
    private double lastSimShotTime = 0.0;
    private Translation3d targetOverride = null;
    private boolean shootOverride = false;

    private final BooleanPublisher isAimEnabledPublisher =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("superstructure/AutoAim/AimEnabled").publish();
    private final BooleanPublisher isShootingEnabledPublisher =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("superstructure/AutoAim/ShootingEnabled").publish();
    private final BooleanPublisher hasValidResultPublisher =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("superstructure/AutoAim/HasValidResult").publish();
    private final BooleanPublisher requiresIdealSpeedPublisher =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("superstructure/AutoAim/RequiresIdealSpeed").publish();
    private final StructArrayPublisher<Pose3d> successPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Flywheel/FuelProjectileSuccessfulShot", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> failPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Flywheel/FuelProjectileUnsuccessfulShot", Pose3d.struct).publish();

    public AutoAimController(
        HoodSubsystem hood,
        TurretSubsystem turret,
        SwerveSubsystem swerve,
        ShooterSubsystem shooter
    ) {
        this.hood = hood;
        this.turret = turret;
        this.swerve = swerve;
        this.shooter = shooter;

        this.turretController = new TurretController(
            turret.getClamp().minAngle.getDegrees(),
            turret.getClamp().maxAngle.getDegrees(),
            0.0,
            0.0,
            Rotation2d.kCW_90deg
        );

        autoAim.setCollisionMap(this::collidesWithHub);
        autoAim.setProcessingCompensation(0.05 + 0.07);
    }

    @Override
    public void periodic() {
        isAimEnabledPublisher.accept(isAimEnabled);
        isShootingEnabledPublisher.accept(isShootingEnabled);

        if (!isAimEnabled) {
            lastResult = null;
            hasValidResultPublisher.accept(false);
            requiresIdealSpeedPublisher.accept(false);
            return;
        }

        lastResult = computeAimResult();

        boolean hasResult = lastResult != null && lastResult.ToF() != -1;
        hasValidResultPublisher.accept(hasResult);
        requiresIdealSpeedPublisher.accept(hasResult && lastResult.requiresIdealSpeed());

        if (!hasResult) return;

        applyAimResult(lastResult);

        if (Robot.isSimulation()) {
            shootSimProjectile(lastResult);
        }
    }

    //#endregion
    //#region Auto-Aim 

    /**
     * Asks the {@link AutoAim} solver for the current result
     *
     * @return A result, or {@code null} if the solver could not find a solution.
     */
    private AutoAimResult computeAimResult() {
        Translation3d target = (targetOverride != null) ? targetOverride : getDefaultTarget();
        return autoAim.calculateDynamicAim(
            swerve.getRelativePose(),
            swerve.getFieldRelativeVelocity(),
            target,
            RPSToMPS(Robot.isSimulation() ? shooter.getCurrentSetpoint() : shooter.getCurrentRPS())
        );
    }

    /**
     * Pushes yaw to the turret and pitch to the hood.
     * Pitch is skipped when time-of-flight is {@code -1}, which the solver
     * uses to signal that the shot cannot land.
     */
    private void applyAimResult(AutoAimResult result) {
        boolean hasValidSpeed = result.recommendedShotSpeed() != -1;
        boolean shouldShoot = isShootingEnabled && shouldAutoShoot(result);
        boolean isUnrestricted = shooter.getShooterClamp() == ShooterClamp.UNRESTRICTED;

        if (hasValidSpeed && (shootOverride || (isUnrestricted && shouldShoot))) {
            shooter.setSetpoint(MPSToRPS(result.recommendedShotSpeed()));
        } else {
            shooter.setSetpoint(0);
        }

        if (!isAimEnabled) return;
        if (result.pitch() != null) {
            hood.setComplexSetpoint(
                Rotation2d.kCCW_Pi_2.minus(result.pitch()), 
                result.pitchOmega() != null ? 
                    result.pitchOmega() : 
                    RotationsPerSecond.of(0)
            );
        }

        if (result.yaw() != null) {
            turret.setComplexSetpoint(
                turretController.calculate(
                    result.yaw(),
                    swerve.getRelativePose().getRotation(),
                    turret.getPosition()
                ),
                result.yawOmega() != null ? 
                    result.yawOmega() : 
                    RotationsPerSecond.of(0)
            );
        }
    }

    private boolean collidesWithHub(Rotation2d pitch, double shotSpeed, boolean usePadding) {
        Translation2d robotPos2d = swerve.getRelativePose().getTranslation()
            .plus(TURRET_TRANSLATION.rotateBy(swerve.getRelativePose().getRotation()));

        Translation2d targetPos2d = new Translation2d(hubPose.getX(), hubPose.getY());

        // Shooter height from ground
        double shooterZ = Units.inchesToMeters(21.443748 + 2.955);

        // Distance to target in XY plane
        double distToTarget = robotPos2d.getDistance(targetPos2d);

        // Projectile horizontal velocity (v_xy) and vertical velocity (v_z)
        double vXY = shotSpeed * pitch.getCos();
        double vZ = shotSpeed * pitch.getSin();

        double halfSide = Units.inchesToMeters(47) / 2.0;

        // Vector from robot to target
        Translation2d toTarget = targetPos2d.minus(robotPos2d);
        Rotation2d angleToTarget = toTarget.getAngle();
        double absCos = Math.abs(angleToTarget.getCos());
        double absSin = Math.abs(angleToTarget.getSin());

        // Distance from center to the square boundary along the shot line
        // We are firing AT the center. The distance from center to edge is determined by
        // which wall we hit first (based on angle).
        // If |cos(theta)| > |sin(theta)|, we hit the vertical walls at x = +/- L/2
        // Else we hit horizontal walls at y = +/- L/2
        double distCenterToWall = (absCos > absSin) ? halfSide / absCos : halfSide / absSin;

        // Distance from robot to the collision wall
        double collisionDist = distToTarget - distCenterToWall;
        if (collisionDist <= 0) return true;

        // Time to travel that horizontal distance
        double t = collisionDist / vXY;

        // Height at that time: z = z0 + vz*t - 0.5*g*t^2
        double zAtCollision = shooterZ + vZ * t - 0.5 * 9.81 * t * t;

        // Check if z is less than hub height. 
        // If it is lower than the rim height when crossing the rim boundary,
        // it hits the side of the hub.
        return zAtCollision < HUB_POSITION.getZ() + (usePadding ? HUB_IDEAL_SHOT_PADDING.in(Meters) : HUB_SHOT_PADDING.in(Meters));
    }

    /**
     * Spawns a simulation projectile at most once every
     * {@link #SIM_SHOT_INTERVAL_SECONDS} seconds so AdvantageScope trajectory
     * visualization stays readable. No-ops when ToF is {@code -1}.
     */
    private void shootSimProjectile(AutoAimResult result) {
        if (result.ToF() == -1) return;
        if ((Timer.getFPGATimestamp() - lastSimShotTime) < SIM_SHOT_INTERVAL_SECONDS) return;

        // lastSimShotTime = Timer.getFPGATimestamp();

        // Rotation2d yaw = result.yaw();
        // if (swerve.shouldFlip()) {
        //     yaw = FieldMirroringUtils.toCurrentAllianceRotation(yaw);
        // }

        // RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
        //     swerve.getRobotPose().getTranslation(),
        //     TURRET_TRANSLATION.rotateBy(swerve.getRobotPose().getRotation()),
        //     swerve.getFieldVelocity(),
        //     yaw,
        //     Meters.of(TURRET_TRANSLATION_3D.getZ()),
        //     MetersPerSecond.of(RPSToMPS(shooter.getCurrentSetpoint())),
        //     Degrees.of(result.pitch().getDegrees())
        // );

        // projectile
        //     .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(getDefaultTarget()))
        //     .withTargetTolerance(new Translation3d(0.67, 0.67, 0.3));

        // projectile.withProjectileTrajectoryDisplayCallBack(
        //     poses -> { successPublisher.set(poses.toArray(Pose3d[]::new)); failPublisher.set(new Pose3d[0]); },
        //     poses -> { failPublisher.set(poses.toArray(Pose3d[]::new)); successPublisher.set(new Pose3d[0]); }
        // );
        // projectile.disableBecomesGamePieceOnFieldAfterTouchGround();

        // SimulatedArena.getInstance().addGamePieceProjectile(projectile);
    }

    //#endregion
    //#region Misc

    private boolean shouldAutoShoot(AutoAimResult result) {
        return (Robot.hubEnabled || Robot.timeUntilSwitch < result.ToF())
            && swerve.getRelativePose().getX() < hubPose.getX();
    }

    private Translation3d getDefaultTarget() {
        if (swerve.getRelativePose().getX() < hubPose.getX()) return BOTTOM_FUNNEL_POSITION;
        return swerve.getRelativePose().getY() < hubPose.getY()
            ? rightPassTarget
            : leftPassTarget;
    }

    /** @return The most recent aim result, or {@code null} if auto-aim is off or unsolved. */
    public AutoAimResult getLastResult() {
        return lastResult;
    }

    //#endregion
    //#region Checks

    public boolean isAimEnabled() {
        return isAimEnabled;
    }

    public boolean hasValidResult() {
        return lastResult != null && lastResult.ToF() != -1;
    }

    /** True when the shot is solvable AND the current flywheel speed is sufficient. */
    public boolean isReadyToShoot() {
        return hasValidResult() && !lastResult.requiresIdealSpeed()
            && (isTurretReady() /*&& isHoodReady()*/);
    }

    public boolean isTurretReady() {
        return Math.abs(
            turret.getPosition().minus(turret.getCurrentSetpoint()).getDegrees()
        ) <= 15.0;
    }

    public boolean isHoodReady() {
        return Math.abs(
            hood.getPosition().minus(hood.getCurrentSetpoint()).getDegrees()
        ) <= 4;
    }

    //#endregion
    //#region Commands
    public Command setAimState(boolean wantAim) {
        return Commands.runOnce(() -> {
            isAimEnabled = wantAim;
        }).ignoringDisable(true);
    }

    public Command setShootingState(boolean wantShoot) {
        return Commands.runOnce(() -> {
            isShootingEnabled = wantShoot;
        }).ignoringDisable(true);
    }

    /** Flips the auto-aim enabled flag. Safe to call while the robot is disabled. */
    public Command aimToggle() {
        return Commands.runOnce(() -> {
            isAimEnabled = !isAimEnabled;
            if (!isAimEnabled) lastResult = null;
        }).ignoringDisable(true);
    }

    /** Flips the auto-shooting enabled flag. Safe to call while the robot is disabled. */
    public Command shootingToggle() {
        return Commands.runOnce(() -> isShootingEnabled = !isShootingEnabled).ignoringDisable(true);
    }

    /** Disables auto-aim and returns the turret and hood to their home positions. */
    public Command reset() {
        return Commands.runOnce(() -> { 
            isAimEnabled = false; 
            lastResult = null; 
        });
    }

    /** Sets a target override for the duration of the command, clearing it on end. */
    public Command overrideTargetCommand(Translation3d target) {
        return Commands.startEnd(() -> targetOverride = target, () -> targetOverride = null);
    }

    public Command overrideShootCommand() {
        return Commands.startEnd(() -> shootOverride = true, () -> shootOverride = false);
    }

    //#endregion
    //#region Conversions

    private double RPSToMPS(double rps) {
        return InchesPerSecond.of(rps * Math.PI * 1.92).in(MetersPerSecond) * (1 - percentLoss);
    }

    private double MPSToRPS(double mps) {
        return MetersPerSecond.of(mps / (1 - percentLoss)).in(InchesPerSecond) / (Math.PI * 1.92);
    }
}
