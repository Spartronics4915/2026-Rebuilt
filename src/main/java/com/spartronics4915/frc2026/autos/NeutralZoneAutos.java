package com.spartronics4915.frc2026.autos;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants;
import com.spartronics4915.frc2026.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//bump/ trench flex. One corner or whole strip of field 
public class NeutralZoneAutos {
    private final SwerveSubsystem swerve;

    public NeutralZoneAutos(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    public Command generateFuelQuadrantCommand(boolean isRightSide) {
        return Commands.defer(() -> {
            Pose2d startOfFuelPose = new Pose2d(
                fuelZoneXAxisPose.plus(startOfFuelTransform.times(isRightSide ? -1 : 1)),
                startOfFuelAngle.rotateBy(Rotation2d.fromDegrees(isRightSide ? 180 : 0))
            );
            Pose2d endOfQuadrantPose = new Pose2d(
                fuelZoneXAxisPose.plus(endQuadrantTransform.times(isRightSide ? -1 : 1)),
                endOfQuadrantAngle.rotateBy(Rotation2d.fromDegrees(isRightSide ? 180 : 0))
            );

            List<Pose2d> poses = new ArrayList<>(List.of(startOfFuelPose, endOfQuadrantPose));

            Autos.addStartingPoseToPath(swerve, poses);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                AutoConstants.defaultPathConstraints,
                Autos.generateStartingState(swerve),
                new GoalEndState(0.0, null)
                //haven't thought about the end state too much, we just want to back to alliance side
                //probs an angle that is generally facing entry area
            );
            
            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }

    public Command generateThroughFuelZoneCommand(boolean isRightSide) {
        return Commands.defer(() -> {
            Pose2d startOfFuelPose = new Pose2d(
                fuelZoneXAxisPose.plus(startOfFuelTransform.times(isRightSide ? -1 : 1)),
                startOfFuelAngle.rotateBy(Rotation2d.fromDegrees(isRightSide ? 180 : 0))
            );
            Pose2d endOfFuelPose = new Pose2d(
                fuelZoneXAxisPose.plus(endOfFuelTransform.times(isRightSide ? -1 : 1)),
                endOfQuadrantAngle.rotateBy(Rotation2d.fromDegrees(isRightSide ? 180 : 0))
            );


            List<Pose2d> poses = new ArrayList<>(List.of(startOfFuelPose, endOfFuelPose));

            Autos.addStartingPoseToPath(swerve, poses);

            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                AutoConstants.defaultPathConstraints,
                Autos.generateStartingState(swerve),
                new GoalEndState(0.0, null)
            );

            return AutoBuilder.followPath(path);
        }, Set.of(swerve));
    }   
}
