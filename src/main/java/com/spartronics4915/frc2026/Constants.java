// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import java.util.ArrayList;
import java.util.List;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class VisionConstants {

        public static final List<LimelightConfiguration> limelightConfigurations = new ArrayList<>();

        public record LimelightConfiguration(
            String name,
            List<Double> robotToCamera
        ) {}

    }
}
