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

// The idea behind this file was so that we could have a cool visualization of the robot driving over the bump,
// but it was never finished, so all it does now is flip the robot 180 when a swerve module is on the bump,
// so that bump autos know when to end (since 180 > tilt tolerance). Sorry I could never finish this - Daniil
public class BumpSim {
    private final Pose2d[] modulePoses;
    StructArrayPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Modules", Pose3d.struct).publish();

    /***
     * Constructor for BumpSim
     * 
     * @param modulePoses the poses of the swerve modules relative to the robot center, in the order of FL, FR, BL, BR
     */
    public BumpSim(Pose2d[] modulePoses) {
        this.modulePoses = modulePoses;
    }

    /***
     * Resolves whether the robot is on the bump and flips it 180 degrees if so.
     * 
     * @param robotPose 2d pose of the robot
     * @return Pose3d of the robot with 180-degree rotation if on bump, otherwise normal pose
     */
    public Pose3d resolveRobotPose(Pose2d robotPose) {
        Translation3d[] globalModulePoses = new Translation3d[modulePoses.length];

        for (int m = 0; m < modulePoses.length; m++) {
            Pose2d modulePose = modulePoses[m];
            Translation2d transModulePose = robotPose.getTranslation().plus(modulePose.getTranslation().rotateBy(robotPose.getRotation()));
            double altitude = getFloorHeight(transModulePose);
            globalModulePoses[m] = new Translation3d(transModulePose.getX(), transModulePose.getY(), altitude);
        }

        // Publish module poses for visualization
        Pose3d[] publishPoses = new Pose3d[globalModulePoses.length];
        for (int i = 0; i < publishPoses.length; i++) {
            publishPoses[i] = new Pose3d(globalModulePoses[i], new Rotation3d());
        }
        posePublisher.accept(publishPoses);

        // If any module is on the bump, flip the robot 180 degrees
        for (int m = 0; m < modulePoses.length; m++) {
            if (globalModulePoses[m].getZ() > 0.01) {
                return new Pose3d(new Pose3d(robotPose).getTranslation().plus(new Translation3d(0, 0, 1)), new Rotation3d(Math.PI, 0, 0));
            }
        }

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

        // Equation for bump height. 6.56 is max height, and is 24.47 inches each way from the center. This is a basic rise over run equation.
        double altitude = 6.56 - 6.56*(distToBump / 24.47);

        return Math.max(Units.inchesToMeters(altitude), 0.0);
    }
}
