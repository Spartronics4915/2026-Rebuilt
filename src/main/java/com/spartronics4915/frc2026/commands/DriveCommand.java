package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
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
 */
public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private final CommandXboxController testingController;

    private static final double maxAngularRate = maxAngularSpeed.in(RadiansPerSecond);

    private Rotation2d lockedHeading = null;

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

        double vX    = applyResponseCurve(MathUtil.applyDeadband(-hid.getLeftY(),  stickDeadband)) * maxSpeed;
        double vY    = applyResponseCurve(MathUtil.applyDeadband(-hid.getLeftX(),  stickDeadband)) * maxSpeed;
        double omega = applyResponseCurve(MathUtil.applyDeadband(-hid.getRightX(), stickDeadband)) * maxAngularRate;

        double override = swerve.getMovementOverride();
        if (override != 0.0) {
            vY = computeOverrideVY(override);
        } else {
            wasOverriding = false;
            yState.position = swerve.getRelativePose().getY();
            yState.velocity = swerve.getFieldVelocity().vyMetersPerSecond;
        }

        if (!swerve.isFieldRelative()) {
            swerve.driveRobotCentric(vX, vY, omega);
            lockedHeading = null;
            return;
        }

        swerve.setDriverPerspective(swerve.getHeadingOffset());

        boolean alignTriggerHeld = hid.getRightTriggerAxis() > 0.5;
        boolean movingFastEnough = Math.hypot(vX, vY) > 0.1;

        if (alignTriggerHeld && movingFastEnough) {
            lockedHeading = new Rotation2d(vX, vY);
            wasAligning = true;
            swerve.driveFieldCentricFacingAngle(vX, vY, lockedHeading);
            return;
        } else if (wasAligning) {
            wasAligning = false;
        }

        double rotationBreakThreshold = (lockedHeading != null) ? maxAngularRate * 0.12 : maxAngularRate * 0.05;
        boolean driverIsRotating = Math.abs(omega) > rotationBreakThreshold;
        boolean driverIsTranslating = Math.hypot(vX, vY) > maxSpeed * 0.05;

        if (driverIsRotating) {
            lockedHeading = null;
            swerve.driveFieldCentric(vX, vY, omega);
        } else if (driverIsTranslating) {
            if (lockedHeading == null) {
                lockedHeading = swerve.getPose().getRotation()
                    .minus(swerve.getHeadingOffset());
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
        goalState.pose = new Pose2d(currentPose.getX(), yState.position, currentPose.getRotation());
        goalState.fieldSpeeds = new ChassisSpeeds();

        ChassisSpeeds robotTarget = overrideController.calculateRobotRelativeSpeeds(currentPose, goalState);
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotTarget, currentPose.getRotation()).vyMetersPerSecond;
    }

    @Override
    public void end(boolean interrupted) {
        wasOverriding = false;
        wasAligning = false;
        lockedHeading = null;
        yState = new TrapezoidProfile.State();
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public boolean runsWhenDisabled() { return false; }

    private XboxController resolveController() {
        if (testingController != null) {
            XboxController hid = testingController.getHID();
            if (hid.isConnected()) return hid;
        }
        return driverController.getHID();
    }

    private static double applyResponseCurve(double x) {
        return Math.signum(x) * (x * x);
    }
    
}
