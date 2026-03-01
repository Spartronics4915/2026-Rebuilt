package com.spartronics4915.frc2026.subsystems.control;

import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static com.spartronics4915.frc2026.Constants.AutoAimConstants.*;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.HoodSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.head.TurretSubsystem;
import com.spartronics4915.frc2026.subsystems.mechanisms.pipeline.ShooterSubsystem;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.AutoAim;
import com.spartronics4915.frc2026.util.AutoAim.AutoAimResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.utils.FieldMirroringUtils;

/**
 * Continuously calculates setpoints and applies them to the
 * hood and turret. Simulation projectile visualization is handled here as well.
 *
 * <p>Auto-aim can be toggled at any time, including while the robot is
 * disabled, via {@link #aimToggle()}. When disabled, the hood and turret retain
 * their last commanded setpoints; call {@link #reset()} to return them home.
 *
 * <p>The calculation pipeline each loop:
 * <ol>
 *   <li>Ask {@link AutoAim} for yaw, pitch, and time-of-flight given the
 *       current robot pose, velocity, and shooter speed.
 *   <li>Push yaw to the turret and pitch to the hood.
 *   <li>If running in simulation, spawn a {@link RebuiltFuelOnFly} projectile
 *       at a throttled rate for trajectory visualization in AdvantageScope.
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
        new Translation3d(
            TURRET_TRANSLATION.getX(),
            TURRET_TRANSLATION.getY(),
            Units.inchesToMeters(21.443748 + 2.955)
        ),
        Rotation2d.fromDegrees(50),
        Rotation2d.fromDegrees(90),
        RPSToMPS(MAX_SHOOTER_RPS)
    );

    private boolean isAimEnabled;
    private boolean isShootingEnabled;
    private AutoAimResult lastResult;
    private double lastSimShotTime;
    private Translation3d targetOverride;

    private final BooleanPublisher isShootingEnabledPublisher =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("superstructure/AutoAim/ShootingEnabled")
            .publish();

    private final BooleanPublisher isAimEnabledPublisher =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("superstructure/AutoAim/AimEnabled")
            .publish();

    private final StructArrayPublisher<Pose3d> successPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Flywheel/FuelProjectileSuccessfulShot", Pose3d.struct)
            .publish();

    private final StructArrayPublisher<Pose3d> failPublisher =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Flywheel/FuelProjectileUnsuccessfulShot", Pose3d.struct)
            .publish();

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
        this.isAimEnabled = false;
        this.isShootingEnabled = false;
        this.targetOverride = null;

        autoAim.setCollisionMap(this::collidesWithHub);
    }

    @Override
    public void periodic() {
        isShootingEnabledPublisher.accept(isAimEnabled);
        isAimEnabledPublisher.accept(isAimEnabled);

        if (!isAimEnabled) {
            lastResult = null;
            return;
        }

        lastResult = computeAimResult();
        if (lastResult == null) return;

        applyAimResult(lastResult);

        if (Robot.isSimulation()) {
            shootSimProjectile(lastResult);
        }
    }

    /**
     * Asks the {@link AutoAim} solver for the current result
     *
     * @return A result, or {@code null} if the solver could not find a solution.
     */
    private AutoAimResult computeAimResult() {
        Translation3d target = (targetOverride != null) ? targetOverride 
            : new Translation3d(hubPose.getX(), hubPose.getY(), HUB_AIM_HEIGHT_METERS);

        return autoAim.calculateDynamicAim(
            swerve.getRelativePose(),
            swerve.getRelativeFieldVelocity(),
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
        if (!isAimEnabled) return;
        if (result.pitch() != null) {
            hood.setSetpoint(Rotation2d.kCCW_Pi_2.minus(result.pitch()));
        }
        turret.setSetpoint(
            result.yaw()
                .minus(swerve.getRelativePose().getRotation())
                .minus(Rotation2d.kCCW_90deg)
        );

        if (!isShootingEnabled && !shouldAutoShoot()) return;
        if (result.recommendedShotSpeed() != -1) {
            shooter.setSetpoint(MPSToRPS(result.recommendedShotSpeed() + 0.1));
        }
    }

    private boolean collidesWithHub(Rotation2d pitch, double shotSpeed) {
        Translation2d robotPos2d = swerve.getRelativePose().getTranslation()
            .plus(TURRET_TRANSLATION.rotateBy(swerve.getRelativePose().getRotation()));
        
        Translation2d targetPos2d = new Translation2d(hubPose.getX(), hubPose.getY());
        
        // Shooter height from ground
        double shooterZ = Units.inchesToMeters(21.443748 + 2.955); // Flywheel top height + fuel radius
        
        // Distance to target in XY plane
        double distToTarget = robotPos2d.getDistance(targetPos2d);
        
        // Projectile horizontal velocity (v_xy) and vertical velocity (v_z)
        double vXY = shotSpeed * pitch.getCos();
        double vZ = shotSpeed * pitch.getSin();
        
        double sideLength = Units.inchesToMeters(47);
        double halfSide = sideLength / 2.0;

        // Vector from robot to target
        Translation2d toTarget = targetPos2d.minus(robotPos2d);
        
        // Angle to target
        Rotation2d angleToTarget = toTarget.getAngle();
        double absCos = Math.abs(angleToTarget.getCos());
        double absSin = Math.abs(angleToTarget.getSin());
        
        // Distance from center to the square boundary along the shot line
        // We are firing AT the center. The distance from center to edge is determined by
        // which wall we hit first (based on angle).
        // If |cos(theta)| > |sin(theta)|, we hit the vertical walls at x = +/- L/2
        // Else we hit horizontal walls at y = +/- L/2
        
        double distCenterToWall;
        if (absCos > absSin) {
            distCenterToWall = halfSide / absCos;
        } else {
            distCenterToWall = halfSide / absSin;
        }

        // Distance from robot to the collision wall
        double collisionDist = distToTarget - distCenterToWall;
        
        if (collisionDist <= 0) return true; 

        // Time to travel that horizontal distance
        double t = collisionDist / vXY;

        // Height at that time: z = z0 + vz*t - 0.5*g*t^2
        double g = 9.81;
        double zAtCollision = shooterZ + vZ * t - 0.5 * g * t * t;

        // Check if z is less than hub height. 
        // If it is lower than the rim height when crossing the rim boundary, it hits the side of the hub.
        return zAtCollision < HUB_AIM_HEIGHT_METERS;
    }

    /**
     * Spawns a simulation projectile at most once every
     * {@link #SIM_SHOT_INTERVAL_SECONDS} seconds so AdvantageScope trajectory
     * visualization stays readable. No-ops when ToF is {@code -1}.
     */
    private void shootSimProjectile(AutoAimResult result) {
        if (result.ToF() == -1) return;
        if ((Timer.getFPGATimestamp() - lastSimShotTime) < SIM_SHOT_INTERVAL_SECONDS) return;

        lastSimShotTime = Timer.getFPGATimestamp();

        Rotation2d yaw = result.yaw();
        if (swerve.shouldFlip()) {
            yaw = FieldMirroringUtils.toCurrentAllianceRotation(yaw);
        }

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
            swerve.getRobotPose().getTranslation(),
            TURRET_TRANSLATION.rotateBy(swerve.getRobotPose().getRotation()),
            swerve.getFieldVelocity(),
            yaw,
            Inches.of(21.443748 + 2.955),
            MetersPerSecond.of(RPSToMPS(shooter.getCurrentSetpoint())),
            Degrees.of(result.pitch().getDegrees())
        );

        projectile
            .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(
                new Translation3d(hubPose.getX(), hubPose.getY(), HUB_DISPLAY_HEIGHT_METERS)
            ))
            .withTargetTolerance(new Translation3d(0.67, 0.67, 0.3));

        projectile.withProjectileTrajectoryDisplayCallBack(
            poses -> { successPublisher.set(poses.toArray(Pose3d[]::new)); failPublisher.set(new Pose3d[0]); },
            poses -> { failPublisher.set(poses.toArray(Pose3d[]::new)); successPublisher.set(new Pose3d[0]); }
        );
        projectile.disableBecomesGamePieceOnFieldAfterTouchGround();

        SimulatedArena.getInstance().addGamePieceProjectile(projectile);
    }

    private boolean shouldAutoShoot() {
        return (Robot.hubEnabled || (!Robot.hubEnabled && Robot.timeUntilSwitch < lastResult.ToF())) 
            && swerve.getRelativePose().getX() < hubPose.getX();
    }

    /** @return The most recent aim solution, or {@code null} if auto-aim is off or unsolved */
    public AutoAimResult getLastResult() {
        return lastResult;
    }

    public boolean isAimEnabled() {
        return isAimEnabled;
    }

    public boolean hasValidResult() {
        if (lastResult == null) return false;
        return (lastResult.ToF() != -1) ? true : false;
    }

    /**
     * Flips the auto-aim enabled flag. Safe to call while the robot is disabled.
     */
    public Command aimToggle() {
        return Commands.runOnce(() -> isAimEnabled = !isAimEnabled).ignoringDisable(true);
    }

    /**
     * Flips the auto-shooting enabled flag. Safe to call while the robot is disabled.
     */
    public Command shootingToggle() {
        return Commands.runOnce(() -> isShootingEnabled = !isShootingEnabled).ignoringDisable(true);
    }

    /**
     * Disables auto-aim and returns the turret and hood to their home positions
     */
    public Command reset() {
        return Commands.parallel(
            Commands.runOnce(() -> isAimEnabled = false),
            turret.setSetpointCommand(Rotation2d.fromDegrees(0)),
            hood.setSetpointCommand(Rotation2d.fromDegrees(0))
        );
    }

    public Command setTargetOverride(Translation3d target) {
        return Commands.runOnce(() -> targetOverride = target);
    }

    public Command clearTargetOverride() {
        return Commands.runOnce(() -> targetOverride = null);
    }

    private double RPSToMPS(double rps) {
        return InchesPerSecond.of(rps * Math.PI * 1.92).in(MetersPerSecond) * (1 - percentLoss);
    }

    private double MPSToRPS(double mps) {
        return MetersPerSecond.of(mps / (1 - percentLoss)).in(InchesPerSecond) / (Math.PI * 1.92);
    }
}
