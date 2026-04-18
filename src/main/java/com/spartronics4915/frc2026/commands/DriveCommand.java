package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.drive.ModeSpeedLimiter;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

/**
 * Default teleop drive command for swerve chassis.
 * Optimized to avoid factory method allocations.
 */
public class DriveCommand extends Command {

    /**
     * Controls how aggressively the driver's speed input is limited while
     * auto-aim is active.
     *
     * <ul>
     *   <li>{@code OFF} – no limit, full driver authority.</li>
     *   <li>{@code HUB} – tight limit for precise hub shots; inspired by 6328's
     *       per-mode speed-capping approach where the robot must be nearly still
     *       to guarantee shot accuracy at close-to-mid range.</li>
     *   <li>{@code FERRY} – looser limit for pass/ferry shots; the target is far
     *       away and can tolerate moderate robot movement without meaningful
     *       aiming error.</li>
     * </ul>
     */
    public enum SpeedLimitMode {
        OFF,
        HUB,
        FERRY
    }

    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private final CommandXboxController testingController;

    private static final double maxAngularRate = maxAngularSpeed.in(RadiansPerSecond);

    private Rotation2d lockedHeading = null;

    private SpeedLimitMode limitMode = SpeedLimitMode.OFF;
    private SpeedLimitMode lastLimitMode = SpeedLimitMode.OFF;

    // Each mode gets its own speed limiter to avoid carrying over incorrect rate when switching
    private final ModeSpeedLimiter hubLimiter = new ModeSpeedLimiter(maxSpeedWhenShootingHub, maxOmegaWhenShootingHub, timeUntilLimitedMaxSpeed, 0.0, 0.0);
    private final ModeSpeedLimiter ferryLimiter = new ModeSpeedLimiter(maxSpeedWhenFerrying, maxOmegaWhenFerrying, timeUntilLimitedMaxSpeed, 0.0, 0.0);

    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trenchAlignConstraints);
    private final TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();
    private final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    private final TrapezoidProfile.State targetState = new TrapezoidProfile.State();
    private TrapezoidProfile.State yState = new TrapezoidProfile.State();
    private boolean wasOverriding = false;

    private boolean wasAligning = false;

    private final PPHolonomicDriveController overrideController =
        new PPHolonomicDriveController(alignTranslationPID, alignRotationPID);

    public DriveCommand(
        CommandXboxController driverController,
        CommandXboxController testingController,
        SwerveSubsystem swerve
    ) {
        this.swerve = swerve;
        this.driverController = driverController;
        this.testingController = testingController;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        XboxController hid = resolveController();

        double vX = applyResponseCurve(MathUtil.applyDeadband(-hid.getLeftY(), stickDeadband)) * maxSpeed;
        double vY = applyResponseCurve(MathUtil.applyDeadband(-hid.getLeftX(), stickDeadband)) * maxSpeed;
        double omega = applyResponseCurve(MathUtil.applyDeadband(-hid.getRightX(), stickDeadband)) * maxAngularRate;

        double override = swerve.getMovementOverride();
        if (override != 0.0) {
            vY = computeOverrideVY(override);
        } else {
            wasOverriding = false;
            yState.position = swerve.getRelativePose().getY();
            yState.velocity = swerve.getFieldVelocity().vyMetersPerSecond;
        }

        if (limitMode != SpeedLimitMode.OFF) {
            // If the mode just changed, reset the limiters for the new mode so we
            // don't snap to the wrong rate or carry over stale state.
            if (limitMode != lastLimitMode) {
                getLimiterFromMode(limitMode).resetAll(vX, vY, omega);
            }

            double[] limited = getLimiterFromMode(limitMode).limit(vX, vY, omega);
                vX = limited[0];
                vY = limited[1];
                omega = limited[2];
        } else {
            // Reset both sets of limiters to current velocity so there is no
            // sudden lurch when a limit mode is first engaged.
            hubLimiter.resetAll(vX, vY, omega);
            ferryLimiter.resetAll(vX, vY, omega);
        }

        lastLimitMode = limitMode;

        if (!swerve.isFieldRelative()) {
            swerve.driveRobotCentric(vX, vY, omega);
            lockedHeading = null;
            return;
        }

        swerve.setDriverPerspective(swerve.getHeadingOffset());

        boolean alignTriggerHeld = false /* hid.getRightTriggerAxis() > 0.5 */;
        boolean movingFastEnough = Math.hypot(vX, vY) > 0.1;

        if (alignTriggerHeld && movingFastEnough) {
            lockedHeading = new Rotation2d(vX, vY);
            wasAligning = true;
            swerve.driveFieldCentricFacingAngle(vX, vY, lockedHeading);
            return;
        } else if (wasAligning) {
            wasAligning = false;
        }

        //double rotationBreakThreshold = (lockedHeading != null) ? maxAngularRate * 0.03 : maxAngularRate * 0.03;
        boolean driverIsRotating = true /* Math.abs(omega) > rotationBreakThreshold */;
        boolean driverIsTranslating = Math.hypot(vX, vY) > maxSpeed * 0.05;

        if (driverIsRotating) {
            lockedHeading = null;
            swerve.driveFieldCentric(vX, vY, omega);
        } else if (driverIsTranslating) {
            if (lockedHeading == null) {
                lockedHeading = swerve.getPose().getRotation().minus(swerve.getHeadingOffset());
            }
            swerve.driveFieldCentricFacingAngle(vX, vY, lockedHeading);
        } else {
            lockedHeading = null;
            swerve.driveFieldCentric(0, 0, 0);
        }
    }

    private double computeOverrideVY(double override) {
        double dt = dtCalc.update();
        ChassisSpeeds fieldVel = swerve.getFieldVelocity();

        if (!wasOverriding) {
            yState.position = swerve.getRelativePose().getY();
            yState.velocity = fieldVel.vyMetersPerSecond;
            wasOverriding = true;
        }

        targetState.position = override;
        targetState.velocity = 0;
        yState = trapezoidProfile.calculate(dt, yState, targetState);

        Pose2d currentPose = swerve.getRelativePose();
        
        goalState.pose = new Pose2d(
            currentPose.getX(), 
            yState.position, 
            currentPose.getRotation()
        );
        
        goalState.fieldSpeeds = new ChassisSpeeds();

        ChassisSpeeds robotTarget = overrideController.calculateRobotRelativeSpeeds(currentPose, goalState);
        
        double cosTheta = currentPose.getRotation().getCos();
        double sinTheta = currentPose.getRotation().getSin();
        
        return robotTarget.vxMetersPerSecond * sinTheta + 
               robotTarget.vyMetersPerSecond * cosTheta;
    }

    /**
     * Sets the active speed-limit mode for this drive command.
     * {@code OFF} removes all limits; {@code HUB} applies a tight cap for precise
     * hub shots; {@code FERRY} applies a looser cap for pass/ferry shots.
     * Safe to call from any periodic context.
     */
    public void setSpeedLimit(SpeedLimitMode mode) {
        limitMode = mode;
    }

    @Override
    public void end(boolean interrupted) {
        wasOverriding = false;
        wasAligning = false;
        lockedHeading = null;
        yState = new TrapezoidProfile.State();
        limitMode = SpeedLimitMode.OFF;
        lastLimitMode = SpeedLimitMode.OFF;
    }

    @Override
    public boolean isFinished() { 
        return false; 
    }

    @Override
    public boolean runsWhenDisabled() { 
        return false; 
    }

    private XboxController resolveController() {
        if (testingController != null) {
            XboxController hid = testingController.getHID();
            if (hid.isConnected()) return hid;
        }
        return driverController.getHID();
    }

    private static double applyResponseCurve(double x) {
        //double ax = Math.abs(x);
        //double linearWeight = 0.60;
        //double shaped = linearWeight * ax + (1.0 - linearWeight) * (ax * ax);
        //return Math.signum(x) * shaped;
        return Math.signum(x) * Math.pow(Math.abs(x), 1.5);
    }

    private ModeSpeedLimiter getLimiterFromMode(SpeedLimitMode mode) {
        return mode == SpeedLimitMode.HUB ? hubLimiter : ferryLimiter;
    }

}