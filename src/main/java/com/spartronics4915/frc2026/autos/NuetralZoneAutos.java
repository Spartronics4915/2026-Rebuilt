package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.endOfFuelTransform;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.endOfQuadrantAngle;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.endQuadrantTransform;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.fuelZoneConstraints;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.fuelZoneXAxisPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.quadrantPathConstraints;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.startOfFuelAngle;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.startOfFuelTransform;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2026.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
//bump/ trench flex. One corner or whole strip of feild 
public class NuetralZoneAutos {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    public NuetralZoneAutos(SwerveSubsystem swerve, VisionSubsystem vision){
        this.swerve = swerve;
        this.vision = vision;
    }

    public Command generateFuelQuadrantCommand(boolean isRightSide){
        Pose2d startOfFuelPose;
        Pose2d endOfQuadrantPose;
        if (isRightSide){
            //magic numbers but is it really necessary to put 180 in constants???
            startOfFuelPose = new Pose2d(fuelZoneXAxisPose.minus(startOfFuelTransform), 
            (startOfFuelAngle.rotateBy(Rotation2d.fromDegrees(180))));
            endOfQuadrantPose = new Pose2d(fuelZoneXAxisPose.minus(endQuadrantTransform), 
            (endOfQuadrantAngle.rotateBy(Rotation2d.fromDegrees(180))));
        } else {
            startOfFuelPose = new Pose2d(fuelZoneXAxisPose.plus(startOfFuelTransform), (startOfFuelAngle));
            endOfQuadrantPose = new Pose2d(fuelZoneXAxisPose.plus(endQuadrantTransform), (endOfQuadrantAngle));   
        }
        List <Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startOfFuelPose,
            endOfQuadrantPose
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            quadrantPathConstraints,
            null,
            new GoalEndState(0.0, null)
            //haven't thought about the end state too much, we just want to back to alliance side
            //probs an angle that is generally facing entry area
        );
        
        return AutoBuilder.followPath(path);
    }

    public Command generateThroughFuelZoneCommand(boolean isRightSide){
        Pose2d startOfFuelPose;
        Pose2d endOfFuelPose;
        if (isRightSide){
            startOfFuelPose = new Pose2d(fuelZoneXAxisPose.minus(startOfFuelTransform), 
            (startOfFuelAngle.rotateBy(Rotation2d.fromDegrees(180))));
            endOfFuelPose = new Pose2d(fuelZoneXAxisPose.minus(startOfFuelTransform),
            (startOfFuelAngle.rotateBy(Rotation2d.fromDegrees(180))));
        } else {
            startOfFuelPose = new Pose2d(fuelZoneXAxisPose.plus(startOfFuelTransform), (startOfFuelAngle));
            endOfFuelPose = new Pose2d(fuelZoneXAxisPose.plus(endOfFuelTransform), (startOfFuelAngle));
        }

        List <Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            startOfFuelPose,
            endOfFuelPose
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            fuelZoneConstraints,
            null,
            new GoalEndState(0.0, null)
        );

        return AutoBuilder.followPath(path);
    }

    
}
