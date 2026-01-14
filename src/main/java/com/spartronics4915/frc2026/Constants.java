// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.cameras.Camera;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class VisionConstants {
        public static final List<Camera> cameraList = new ArrayList<>();

        public enum CameraType {
            LUMA, LIMELIGHT
        }

        public static final SimCameraProperties simCameraProperties = new SimCameraProperties();
            static {
                simCameraProperties.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
                simCameraProperties.setCalibError(0.12, 0.04);
                simCameraProperties.setFPS(60);
                simCameraProperties.setAvgLatencyMs(15);
                simCameraProperties.setLatencyStdDevMs(5);
            }
    }

}
