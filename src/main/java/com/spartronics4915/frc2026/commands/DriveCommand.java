package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;
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

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

/**
 * Default teleop drive command for swerve chassis.
 *
 * <p>Intentionally imports no CTRE types — all drive calls go through
 * {@link SwerveSubsystem}'s named API so this command is insulated from
 * any future drivetrain library changes.
 *
 * <p>Features:
 * <ul>
 *   <li>Cubic response curve for finer low-speed control
 *   <li>Heading lock: when the rotation stick is released the robot actively
 *       holds its last heading via {@link SwerveSubsystem#driveFieldCentricFacingAngle}
 *   <li>Movement override: trapezoidal Y-axis snap for trench alignment
 *   <li>Debug controller hot-swap: if a second controller is connected it
 *       takes over input automatically
 * </ul>
 */
public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private final CommandXboxController testingController;

    private static final double maxAngularRate = maxAngularSpeed.in(RadiansPerSecond);

    // Heading lock — null means no lock is active yet.
    private Rotation2d lockedHeading = null;

    private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trenchAlignConstraints);
    private final TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();
    private final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    private final TrapezoidProfile.State targetState = new TrapezoidProfile.State();
    private TrapezoidProfile.State yState = new TrapezoidProfile.State();
    private boolean wasOverriding = false;

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

        // Movement override replaces Y velocity for trench alignment.
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

        boolean driverIsRotating = Math.abs(omega) > maxAngularRate * 0.05;
        if (driverIsRotating) {
            // Driver is actively rotating — clear the lock and drive normally.
            lockedHeading = null;
            swerve.driveFieldCentric(vX, vY, omega);
        } else {
            // Rotation stick released — engage heading lock.
            if (lockedHeading == null) {
                lockedHeading = swerve.getRelativePose().getRotation();
            }
            swerve.driveFieldCentricFacingAngle(vX, vY, lockedHeading);
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

        ChassisSpeeds robotTarget = driveController.calculateRobotRelativeSpeeds(currentPose, goalState);
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotTarget, currentPose.getRotation()).vyMetersPerSecond;
    }

    @Override
    public void end(boolean interrupted) {
        wasOverriding = false;
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
        return x * x * x;
    }
    
}