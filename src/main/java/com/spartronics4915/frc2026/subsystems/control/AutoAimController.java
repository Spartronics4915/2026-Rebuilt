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
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Continuously calculates setpoints and applies them to the hood and turret.
 */
public class AutoAimController extends SubsystemBase {

    private final HoodSubsystem hood;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final SwerveSubsystem swerve;

    private final AutoAim autoAim = new AutoAim(
        20, // 30
        50, // 100
        0.001,
        turretTranslation3D,
        Rotation2d.fromDegrees(50),
        Rotation2d.fromDegrees(90),
        RPSToMPS(MAX_SHOOTER_RPS),
        0.02,
        this::collidesWithHub,
        this::collidesWithHubWithPadding
    );

    private final Debouncer possibleDebouncer = new Debouncer(PIPELINE_RATE_LIMIT_SEC, DebounceType.kFalling);
    private boolean isPossibleDebouncedValue = false;

    private final TurretController turretController;

    private boolean isAimEnabled = false;
    private boolean isAutoShootingEnabled = false;
    private AutoAimResult lastResult = null;
    private Translation3d targetOverride = null;
    private boolean shootOverride = false;

    public enum ManualOverride {
        LEFT(Rotation2d.fromDegrees(-127.885), Rotation2d.fromDegrees(28.12), 8.382),
        RIGHT(Rotation2d.fromDegrees(-48.015), Rotation2d.fromDegrees(28.78), 8.447);

        public final Rotation2d yaw;
        public final Rotation2d pitch;
        public final double speedMPS;

        ManualOverride(Rotation2d yaw, Rotation2d pitch, double speedMPS) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.speedMPS = speedMPS;
        }
    }
    
    //private int manualOverrideIndex = 0;
    private ManualOverride activeManualOverride = null;
    
    private final TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();
    private ChassisSpeeds lastFieldSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldAccelerations = new ChassisSpeeds();
    
    private final LinearFilter accelFilterX = LinearFilter.movingAverage(5);
    private final LinearFilter accelFilterY = LinearFilter.movingAverage(5);
    private final LinearFilter accelFilterOmega = LinearFilter.movingAverage(5);

    private final LinearFilter flywheelFilter = LinearFilter.movingAverage(15);

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
            0.01,
            Rotation2d.kCW_90deg.plus(Rotation2d.fromDegrees(180))
        );

        turretController.reset(turret.getPosition());
    }

    // Collision cache
    private Double cachedCollisionDist = null;

    @Override
    public void periodic() {
        isAimEnabledPublisher.accept(isAimEnabled);
        isShootingEnabledPublisher.accept(isAutoShootingEnabled);

        isPossibleDebouncedValue = possibleDebouncer.calculate(isPhysicallyAligned());

        double dt = Math.max(dtCalc.update(), 0.001); // Prevent division by zero
        ChassisSpeeds currentSpeeds = swerve.getFieldRelativeVelocity();

        double rawAccelX = (currentSpeeds.vxMetersPerSecond - lastFieldSpeeds.vxMetersPerSecond) / dt;
        double rawAccelY = (currentSpeeds.vyMetersPerSecond - lastFieldSpeeds.vyMetersPerSecond) / dt;
        double rawAccelOmega = (currentSpeeds.omegaRadiansPerSecond - lastFieldSpeeds.omegaRadiansPerSecond) / dt;

        fieldAccelerations = new ChassisSpeeds(
            accelFilterX.calculate(rawAccelX),
            accelFilterY.calculate(rawAccelY),
            accelFilterOmega.calculate(rawAccelOmega)
        );
        lastFieldSpeeds = currentSpeeds;

        updateCollisionCache();

        if (isAimEnabled) {
            lastResult = computeAimResult();
        } else {
            lastResult = null;
            flywheelFilter.reset();
        }

        boolean hasResult = lastResult != null && lastResult.ToF() != -1;
        hasValidResultPublisher.accept(hasResult);
        requiresIdealSpeedPublisher.accept(hasResult && lastResult.requiresIdealSpeed());

        if (activeManualOverride != null) {
            if (shootOverride) {
                shooter.setSetpoint(MPSToRPS(activeManualOverride.speedMPS));
                hood.setSetpoint(activeManualOverride.pitch);
            } else {
                shooter.setSetpoint(0);
                hood.setSetpoint(Rotation2d.kZero);
            }
            turret.setSetpoint(activeManualOverride.yaw);
            return;
        }

        if (!isAimEnabled) {
            return;
        }

        if (!hasResult) return;

        applyAimResult(lastResult);
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
        
        if (!isPassTarget()) {
            autoAim.setCollisionMap(this::collidesWithHub, this::collidesWithHubWithPadding);
        } else {
            autoAim.setCollisionMap(null, null);
        }

        return autoAim.calculateDynamicAim(
            swerve.getSmoothedRelativePose(),
            lastFieldSpeeds,
            fieldAccelerations,
            target,
            RPSToMPS(flywheelFilter.calculate(Robot.isSimulation() ? shooter.getCurrentSetpoint() : shooter.getCurrentRPS())),
            0.08 // Dynamic processing compensation can be supplied here
        );
    }

    /**
     * Pushes yaw to the turret and pitch to the hood.
     * Pitch is skipped when time-of-flight is {@code -1}, which the solver
     * uses to signal that the shot cannot land.
     */
    private void applyAimResult(AutoAimResult result) {
        boolean readyToShoot = meetsFiringConditions(result);
        if (readyToShoot) {
            shooter.setSetpoint(MPSToRPS(result.recommendedShotSpeed() * (isPassTarget() ? 1.05 : 1)));
        } else {
            shooter.setSetpoint(0);
        }

        if (!isAimEnabled) return;
        if (result.pitch() != null && shooter.getCurrentSetpoint() != 0) {
            // This might be a little iffy because of flywheel losing speed
            hood.setComplexSetpoint(
                Rotation2d.kCCW_Pi_2.minus(result.pitch()), 
                result.pitchOmega() != null ? 
                    result.pitchOmega() : 
                    RotationsPerSecond.of(0)
            );
        } else {
            hood.setSetpoint(Rotation2d.kZero);
        }

        if (!readyToShoot) {
            result = autoAim.calculateDynamicAim(
                swerve.getSmoothedRelativePose(),
                new ChassisSpeeds(0.0, 0.0, lastFieldSpeeds.omegaRadiansPerSecond),
                new ChassisSpeeds(0.0, 0.0, fieldAccelerations.omegaRadiansPerSecond),
                BOTTOM_FUNNEL_POSITION,
                0.0,
                0.08
            );
        }

        if (result.yaw() != null) {
            Rotation2d recommendedAngle = null;
            if (DriverStation.isAutonomous()) {
                if (swerve.getRelativePose().getY() > hubPose.getY()) {
                    recommendedAngle = Rotation2d.fromDegrees(-45);
                } else {
                    recommendedAngle = Rotation2d.fromDegrees(45);
                }
            }

            turret.setComplexSetpoint(
                turretController.calculate(
                    result.yaw(),
                    swerve.getRelativePose().getRotation(),
                    turret.getPosition(),
                    recommendedAngle
                ),
                result.yawOmega() != null ? 
                    result.yawOmega() : 
                    RotationsPerSecond.of(0)
            );
        }
    }

    private boolean collidesWithHub(Rotation2d pitch, double shotSpeed) {
        return checkHubCollision(pitch, shotSpeed, false);
    }

    private boolean collidesWithHubWithPadding(Rotation2d pitch, double shotSpeed) {
        return checkHubCollision(pitch, shotSpeed, true);
    }

    private boolean checkHubCollision(Rotation2d pitch, double shotSpeed, boolean usePadding) {
        if (cachedCollisionDist == null) return false;
        double collisionDist = cachedCollisionDist;

        if (collisionDist <= 0) return true;

        // Shooter height from ground
        double shooterZ = Units.inchesToMeters(21.443748 + 2.955);

        // Projectile horizontal velocity (v_xy) and vertical velocity (v_z)
        double vXY = shotSpeed * pitch.getCos();
        double vZ = shotSpeed * pitch.getSin();

        // Time to travel that horizontal distance
        double t = collisionDist / vXY;

        // Height at that time
        double zAtCollision = shooterZ + vZ * t - 0.5 * 9.81 * t * t;

        // Check if z is less than hub height. 
        // If it is lower than the rim height when crossing the rim boundary,
        // it hits the side of the hub.
        return zAtCollision < HUB_POSITION.getZ() + (usePadding ? HUB_IDEAL_SHOT_PADDING.in(Meters) : HUB_SHOT_PADDING.in(Meters));
    }

    private void updateCollisionCache() {
        Translation2d robotPos2d = swerve.getSmoothedRelativePose().getTranslation()
            .plus(turretTranslation2D.rotateBy(swerve.getRelativePose().getRotation()));

        Translation2d targetPos2d = new Translation2d(hubPose.getX(), hubPose.getY());

        double distToTarget = robotPos2d.getDistance(targetPos2d);
        double halfSide = Units.inchesToMeters(47) / 2.0;

        Translation2d toTarget = targetPos2d.minus(robotPos2d);
        Rotation2d angleToTarget = toTarget.getAngle();
        double absCos = Math.abs(angleToTarget.getCos());
        double absSin = Math.abs(angleToTarget.getSin());

        double distCenterToWall = (absCos > absSin) ? halfSide / absCos : halfSide / absSin;
        cachedCollisionDist = distToTarget - distCenterToWall;
    }

    //#endregion
    //#region Misc

    private boolean shouldAutoShoot(AutoAimResult result) {
        return (Robot.hubEnabled || Robot.timeUntilSwitch < result.ToF())
            && swerve.getRelativePose().getX() < hubPose.getX()
            && swerve.isFlatDebounced();
    }

    public boolean isPassTarget() {
        Translation3d target = (targetOverride != null) ? targetOverride : getDefaultTarget();
        return !target.equals(BOTTOM_FUNNEL_POSITION);
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

    public boolean isPhysicallyAligned() {
        return lastResult != null && !lastResult.requiresIdealSpeed()
            && isHoodReady()
            && isTurretReady();
    }

    public boolean isPhysicallyAlignedDebounced() {
        return isPossibleDebouncedValue;
    }

    public boolean meetsFiringConditions(AutoAimResult result) {
        boolean hasValidSpeed = result.recommendedShotSpeed() != -1;
        boolean shouldShoot = isAutoShootingEnabled && shouldAutoShoot(result);
        boolean isUnrestricted = shooter.getShooterClamp() == ShooterClamp.UNRESTRICTED;

        boolean readyToShoot = hasValidSpeed && (shootOverride || (isUnrestricted && shouldShoot));
        return readyToShoot;
    }

    /** True when the shot is solvable AND the current flywheel speed is sufficient. */
    public boolean isReadyToShoot() {
        // Check if the manual shooter is within the allowed leniency as well as that it's commanded to shoot (since we can't check with the auto-aim system if the shot is possible)
        if (activeManualOverride != null && shootOverride) {
            return shooter.getCurrentRPS() / shooter.getCurrentSetpoint() >= 1.0 - manualShooterLeniency;
        }

        // General case when auto-aim is enabled, it has to have a valid result and speed, and the turret and hood have to be near their setpoints
        if (hasValidResult() && isPhysicallyAlignedDebounced()
            // Hood and turret moved to isPhysicallyAligned because they move A LOT
            && !turretController.isWrapping()
            && meetsFiringConditions(lastResult)
        ) {
            return true;
        }
 
        return false;
    }

    // TODO: This could not be working

    public boolean isTurretReady() {
        if (turretController.isWrapping()) return false;

        return Math.abs(
            Math.IEEEremainder( 
                turret.getPosition().getDegrees() 
                - turretController.getLastSetpoint(), 360.0
            )
        ) <= (isPassTarget() ? 15.0 : 4.0);
    }

    public boolean isHoodReady() {
        return Math.abs(
            hood.getPosition().minus(hood.getCurrentSetpoint()).getDegrees()
        ) <= (isPassTarget() ? 15.0 : 5.0);
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
            isAutoShootingEnabled = wantShoot;
        }).ignoringDisable(true);
    }

    /** Flips the auto-aim enabled flag. Safe to call while the robot is disabled. */
    public Command aimToggle() {
        return Commands.runOnce(() -> {
            isAimEnabled = !isAimEnabled;
        }).ignoringDisable(true);
    }

    /** Flips the auto-shooting enabled flag. Safe to call while the robot is disabled. */
    public Command shootingToggle() {
        return Commands.runOnce(() -> isAutoShootingEnabled = !isAutoShootingEnabled).ignoringDisable(true);
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
        return Commands.startEnd(
            () -> shootOverride = true,
            () -> {
                shootOverride = false;
                shooter.setSetpoint(0);
            }
        );
    }

    public Command setManualOverride(ManualOverride override) {
        return Commands.startEnd(
            () -> activeManualOverride = override,
            () -> activeManualOverride = null
        ).ignoringDisable(true);
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
