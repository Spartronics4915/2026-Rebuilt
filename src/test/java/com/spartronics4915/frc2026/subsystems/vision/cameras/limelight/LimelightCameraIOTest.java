package com.spartronics4915.frc2026.subsystems.vision.cameras.limelight;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

class LimelightCameraIOTest {
    @Test
    void correctsReportedPoseToCaptureTimeTurretTransform() {
        Pose3d actualRobotPose = new Pose3d(
            4.0,
            2.0,
            0.0,
            new Rotation3d(0.0, 0.0, Math.toRadians(25.0)));
        Transform3d transformAtCapture = new Transform3d(
            new Translation3d(0.35, 0.20, 0.0),
            new Rotation3d(0.0, 0.0, Math.toRadians(70.0)));
        Transform3d configuredTransform = new Transform3d(
            new Translation3d(0.45, -0.10, 0.0),
            new Rotation3d(0.0, 0.0, Math.toRadians(20.0)));

        Pose3d fieldToCamera = actualRobotPose.transformBy(transformAtCapture);
        Pose2d incorrectlyReportedRobotPose = fieldToCamera
            .transformBy(configuredTransform.inverse())
            .toPose2d();

        Pose3d corrected = LimelightCameraIO.correctRobotPose(
            incorrectlyReportedRobotPose,
            configuredTransform,
            transformAtCapture);

        assertEquals(actualRobotPose.getX(), corrected.getX(), 1e-9);
        assertEquals(actualRobotPose.getY(), corrected.getY(), 1e-9);
        assertEquals(
            actualRobotPose.getRotation().getZ(),
            corrected.getRotation().getZ(),
            1e-9);
    }
}
