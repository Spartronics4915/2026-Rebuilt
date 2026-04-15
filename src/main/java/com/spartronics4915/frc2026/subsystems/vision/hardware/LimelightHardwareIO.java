package com.spartronics4915.frc2026.subsystems.vision.hardware;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicReference;

import com.spartronics4915.frc2026.Constants.VisionConstants;
import com.spartronics4915.frc2026.subsystems.vision.samples.FiducialObservation;
import com.spartronics4915.frc2026.subsystems.vision.samples.PoseEstimate;
import com.spartronics4915.frc2026.util.vision.LimelightHelpers;

/** Hardware implementation of VisionIO using Limelight cameras. */
public class LimelightHardwareIO implements VisionIO {
    NetworkTable tableC = NetworkTableInstance.getDefault().getTable(VisionConstants.cameraCTableName);
    AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());
    int imuMode = 1;

    private static final double[] default_std_devs = new double[VisionConstants.expectedStdDevLength];

    /** Creates a new Limelight vision IO instance. */
    public LimelightHardwareIO() {
        setCameraSettings();
    }

    /** Configures Limelight camera poses in robot coordinate system. */
    private void setCameraSettings() {
        double[] cameraCPose = {
            VisionConstants.robotToCameraCForward,
            VisionConstants.robotToCameraCSide,
            VisionConstants.cameraCHeightOffGroundMeters,
            0.0,
            VisionConstants.cameraCPitchDegrees,
            VisionConstants.cameraCYawOffset.getDegrees()
        };

        tableC.getEntry("cameraPose_robotSpace_set").setDoubleArray(cameraCPose);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(tableC, inputs.cameraC, VisionConstants.cameraCTableName);
        latestInputs.set(inputs);
    }

    /** Reads data from a single Limelight camera. */
    private void readCameraData(NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        if (camera.seesTarget) {
            try {
                LimelightHelpers.PoseEstimate megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                Pose3d robotPose3d =
                    LimelightHelpers.toPose3D(
                        LimelightHelpers.getBotPose_wpiBlue(limelightName)
                    );

                if (megatag != null) {
                    camera.poseEstimate = PoseEstimate.fromLimelight(megatag);
                    camera.poseCount = megatag.tagCount;
                    camera.fiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
                }

                if (robotPose3d != null) {
                    camera.pose3d = robotPose3d;
                }

                camera.standardDeviations = table.getEntry("std_devs").getDoubleArray(default_std_devs);
            } catch (Exception e) {
                System.err.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }
    
}