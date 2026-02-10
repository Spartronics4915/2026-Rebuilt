package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.endTriggerDebounce;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.positionTolerance;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.rotationTolerance;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.speedTolerance;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.spartronics4915.frc2026.Constants.SwerveConstants;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PositionPIDCommand extends Command{
    
    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = SwerveConstants.AutoConstants.autoAlignPIDController;

    private final Timer timer = new Timer();

    private final Debouncer endTriggerDebouncer = new Debouncer(endTriggerDebounce.in(Seconds));

    private final DoublePublisher xErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("X Error").publish();
    private final DoublePublisher yErrLogger = NetworkTableInstance.getDefault().getTable("logging").getDoubleTopic("Y Error").publish();



    private PositionPIDCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.drive(new ChassisSpeeds(0,0,0));
            swerve.lockModules();
        });
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        mSwerve.drive(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getPose(), goalState
            )
        );

        xErrLogger.accept(mSwerve.getPose().getX() - goalPose.getX());
        yErrLogger.accept(mSwerve.getPose().getY() - goalPose.getY());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        System.out.println("Adjustments to alignment took: " + timer.get() + " seconds and interrupted = " + interrupted
            + "\nPosition offset: " + Centimeter.convertFrom(diff.getTranslation().getNorm(), Meters) + " cm"
            + "\nRotation offset: " + diff.getRotation().getMeasure().in(Degrees) + " deg"
            + "\nVelocity value: " + mSwerve.getSpeed() + "m/s"
        );
    }

    @Override
    public boolean isFinished() {

        Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

        var rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            rotationTolerance.getRotations(), 
            0.0, 
            1.0
        );

        var position = diff.getTranslation().getNorm() < positionTolerance.in(Meters);

        var speed = mSwerve.getSpeed() < speedTolerance.in(MetersPerSecond);

        // System.out.println("end trigger conditions R: "+ rotation + "\tP: " + position + "\tS: " + speed);
        
        return endTriggerDebouncer.calculate(
            rotation && position && speed
        );
    }
}
