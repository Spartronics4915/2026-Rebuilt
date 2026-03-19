package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

/**
 * Default teleop drive command for swerve chassis.
 * Owns all controller input processing, movement override logic, 
 * and trajectory calculation.
 */
public class DriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private final CommandXboxController testingController;

    private static final double maxAngularRate = maxAngularSpeed.in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1);

    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1);

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

        double vX = applyResponseCurve(MathUtil.applyDeadband(hid.getLeftY() * -1.0, stickDeadband)) * maxSpeed;
        double vY = applyResponseCurve(MathUtil.applyDeadband(hid.getLeftX() * -1.0, stickDeadband)) * maxSpeed;
        double omega = applyResponseCurve(MathUtil.applyDeadband(hid.getRightX() * -1.0, stickDeadband)) * maxAngularRate;

        double override = swerve.getMovementOverride();
        if (override != 0.0) {
            vY = computeOverrideVY(override);
        } else {
            wasOverriding = false;
            yState.position = swerve.getPose().getY();
            yState.velocity = swerve.getFieldVelocity().vyMetersPerSecond;
        }
        if (swerve.isFieldRelative) {
            swerve.drivetrain.setOperatorPerspectiveForward(swerve.teleopHeadingOffset);
            swerve.drivetrain.setControl(
                fieldCentricRequest
                    .withVelocityX(vX)
                    .withVelocityY(vY)
                    .withRotationalRate(omega)
            );
        } else {
            swerve.drivetrain.setControl(
                robotCentricRequest
                    .withVelocityX(vX)
                    .withVelocityY(vY)
                    .withRotationalRate(omega)
            );
        }
    }

    private double computeOverrideVY(double override) {
        double dt = dtCalc.update();
        ChassisSpeeds fieldVel = swerve.getFieldVelocity();

        if (!wasOverriding) {
            yState.position = swerve.getPose().getY();
            yState.velocity = fieldVel.vyMetersPerSecond;
            wasOverriding = true;
        }

        targetState.position = override;
        targetState.velocity = 0;
        yState = trapezoidProfile.calculate(dt, yState, targetState);

        Pose2d currentPose = swerve.getPose();
        goalState.pose = new Pose2d(currentPose.getX(), yState.position, currentPose.getRotation());
        goalState.fieldSpeeds = new ChassisSpeeds();

        ChassisSpeeds robotTarget = driveController.calculateRobotRelativeSpeeds(currentPose, goalState);
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotTarget, currentPose.getRotation()).vyMetersPerSecond;
    }

    @Override
    public void end(boolean interrupted) {
        wasOverriding = false;
        yState = new TrapezoidProfile.State();
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
        return Math.signum(x) * (x * x);
    }

}