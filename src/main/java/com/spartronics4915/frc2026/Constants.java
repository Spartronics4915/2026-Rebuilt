// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    public static class HoodConstants {
        public static final double P = 250.0;
        public static final double I = 0.0;
        public static final double D = 0.05;

        public static final double HOOD_MAX_VELOCITY = 10;
        public static final double HOOD_MAX_ACCELERATION = 10;

        public static final double HOOD_DT = 0.02;

        public static final int HOOD_MOTOR_ID = 21;

        public static final boolean HOOD_CURRENT_LIMIT_ENABLE = true;
        public static final double HOOD_CURRENT_LIMIT = 40;
        public static final double HOOD_LOWER_LIMIT = 20;

        public static final double HOOD_LOWER_TIME = 1;
        public static final double HOOD_SENSOR_MECHANISM_RATIO = 85.3333333;

        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(30);
    }
}
