package com.spartronics4915.frc2026.subsystems.vision.hardware;

import com.spartronics4915.frc2026.subsystems.vision.samples.FiducialObservation;
import com.spartronics4915.frc2026.subsystems.vision.samples.PoseEstimate;

import edu.wpi.first.math.geometry.Pose3d;

/** Interface for vision system hardware abstraction. */
public interface VisionIO {

    /** Container for all vision input data. */
    class VisionIOInputs {
        /** Input data from a single camera. */
        public static class CameraInputs {
            public boolean seesTarget;
            public FiducialObservation[] fiducialObservations;
            public PoseEstimate poseEstimate;
            public int poseCount;
            public Pose3d pose3d;
            public double[] standardDeviations = new double[6]; // [x, y, z, roll, pitch, Yaw]
        }

        public CameraInputs cameraA = new CameraInputs();
        public CameraInputs cameraB = new CameraInputs();
        public CameraInputs cameraC = new CameraInputs();
    }

    void readInputs(VisionIOInputs inputs);
    
}