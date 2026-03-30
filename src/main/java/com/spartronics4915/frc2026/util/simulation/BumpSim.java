package com.spartronics4915.frc2026.util.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.bumpTrenchDivTransform;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.centerPose;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.hubPose;

import java.util.Arrays;

public class BumpSim {
    private final Pose2d[] modulePoses;
    private final int maxIterations;
    private final double convergenceThreshold;
    StructArrayPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Modules", Pose3d.struct).publish();

    /***
     * Constructor for BumpSim
     * 
     * @param modulePoses
     *            the poses of the swerve modules relative to the robot center, in the order of FL, FR, BL, BR
     * @param maxIterations
     *            the maximum number of iterations to run the simulation for, to prevent infinite loops. Should be set high enough to
     *            allow convergence, but low enough to prevent long runtimes in edge cases.
     * @param convergenceThreshold
     *            the threshold for determining convergence of swerve angle in degrees
     */
    public BumpSim(Pose2d[] modulePoses, int maxIterations, double convergenceThreshold) {
        this.modulePoses = modulePoses;
        this.maxIterations = maxIterations;
        this.convergenceThreshold = convergenceThreshold;
    }

    /***
     * Calculates the pose of the robot in 3d based on the 2d pose of the robot. Runs for a certain number of iterations, as the
     * projected swerve modules in 2d get closer together as the tilt of the robot increases. Only call this in Simulation, and use the
     * IMU otherwise.
     * 
     * @param robotPose 
     *            2d pose of the robot
     * @return Pose3d of the robot, with the Z value representing the estimated height of the robot based on swerve module heights
     */
    public Pose3d resolveRobotPose(Pose2d robotPose) {
        Rotation3d currentRotation = new Rotation3d();

        // for (int i = 0; i < maxIterations; i++) {
        Translation3d[] globalModulePoses = new Translation3d[modulePoses.length];
        int highestModuleIndex = -1;

        for (int m = 0; m < modulePoses.length; m++) {
            Pose2d modulePose = modulePoses[m];
            Translation3d localPose = new Translation3d(modulePose.getX(), modulePose.getY(), 0);
            Translation3d rotatedPose = localPose.rotateBy(currentRotation);
            Translation2d transModulePose = new Translation2d(rotatedPose.getX(), rotatedPose.getY());
            transModulePose = robotPose.getTranslation().plus(transModulePose.rotateBy(robotPose.getRotation()));

            double altitude = getFloorHeight(transModulePose);
            globalModulePoses[m] = new Translation3d(transModulePose.getX(), transModulePose.getY(), altitude);

            // if (highestModuleIndex == -1 || altitude > globalModulePoses[highestModuleIndex].getZ()) {
            //     highestModuleIndex = m;
            // }
        }

        // int modulePair = (highestModuleIndex + (highestModuleIndex % 2 == 0 ? 1 : -1) + modulePoses.length) % modulePoses.length; // Pair modules are FL-BR and FR-BL
        Pose3d[] publishPoses = new Pose3d[globalModulePoses.length];
        for (int i = 0; i < publishPoses.length; i++) {
            publishPoses[i] = new Pose3d(globalModulePoses[i], new Rotation3d());
        }
        posePublisher.accept(publishPoses);

        for (int m = 0; m < modulePoses.length; m++) {
            if (globalModulePoses[m].getZ() > 0.01) {
                return new Pose3d(new Pose3d(robotPose).getTranslation().plus(new Translation3d(0, 0, 1)), new Rotation3d(Math.PI, 0, 0));
            }
        }

        // }
        return new Pose3d(robotPose);
    }

    public double getFloorHeight(Translation2d coordinates) {
        if (Math.abs(coordinates.minus(hubPose).getY()) > bumpTrenchDivTransform.getY()) {
            return 0.0; // Bump doesn't span into trench area
        }

        Translation2d closestHub;
        if (coordinates.minus(centerPose).getX() < 0) {
            closestHub = hubPose;
        } else {
            closestHub = centerPose.minus(hubPose).times(2).plus(hubPose); // Mirror across center to get the red hub
        }

        double distToBump = Units.metersToInches(Math.abs(coordinates.minus(closestHub).getX()));

        double altitude = 6.56 - 6.56*(distToBump / 24.47);

        return Math.max(Units.inchesToMeters(altitude), 0.0);
    }
}
