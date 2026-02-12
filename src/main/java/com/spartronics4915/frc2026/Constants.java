// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    public static class TurretConstants{
        public static final int TURRET_MOTOR_ID = 19;

        public static final double TURRET_P = 5.0;
        public static final double TURRET_I = 0.0;
        public static final double TURRET_D = 0.0;

        public static final boolean CURRENT_LIMIT_ENABLED = true;
        public static final double SUPPLY_CURRENT_LIMIT = 30;
        public static final double CURRENT_LOWER_LIMIT = 15;
        public static final double CURRENT_LOWER_TIME = 1.0;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1;
        public static final double MIN_ROTATION = -1; //The Turret Subsystem is currently written using rotations as the values in its pid, so this is equivalent to -360 degrees
        public static final double MAX_ROTATION = 1;

        public static final double MAX_VELOCITY = 4;
        public static final double MAX_ACCELERATION = 2;
        public static final double DELTA_TIME = 1.0/50.0;

        public static final int ENCODER_ONE_ID = 0;
        public static final int ENCODER_TWO_ID = 0;
    }
}
