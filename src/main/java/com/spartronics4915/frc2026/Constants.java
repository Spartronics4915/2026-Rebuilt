// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    public static class HoodConstants {
        public static final double P = 5.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double HOOD_MAX_VELOCITY = 15.0;
        // If max acceleration is put higher than like 5 then it may jump super hard when first enabled while going to initial 0 (how fix)
        public static final double HOOD_MAX_ACCELERATION = 5;
        public static final double HOOD_MIN = 0.0;
        public static final double HOOD_MAX = 90.0;
        public static final double HOOD_DT = 1.0/50.0;
        public static final int HOOD_MOTOR_ID = 3;
        public static final boolean HOOD_CURRENT_LIMIT_ENABLE = true;
        public static final double HOOD_CURRENT_LIMIT = 40;
        public static final double HOOD_LOWER_LIMIT = 20;
        public static final double HOOD_LOWER_TIME = 1;
        public static final double HOOD_SENSOR_MECHANISM_RATIO = 1/1;
    }
}
