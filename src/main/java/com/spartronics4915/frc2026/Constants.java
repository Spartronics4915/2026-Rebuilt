// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 3;
        public static final double INTAKE_MAX_VELOCITY = 50.0;
        public static final double INTAKE_MAX_ACCELERATION = 1.0;
        public static final double INTAKE_POSITION = 0.0;
        public static final double INTAKE_VELOCITY = 0.0;
        public static final double INTAKE_MINIMUM_VELOCITY = 0.0;
        public static final double INTAKE_MAXIMUM_VELOCITY = 50.0;
        public static final double INTAKE_DT = 1.0/50.0;

        public static final double INTAKE_P = 0.02;
        public static final double INTAKE_I = 0;
        public static final double INTAKE_D = 0;

        public static final boolean INTAKE_CURRENT_LIMIT_ENABLE = true;
        public static final double INTAKE_CURRENT_LIMIT = 30;
        public static final double INTAKE_CURRENT_LOWER_LIMIT = 10;
        public static final double INTAKE_CURRENT_LOWER_TIME = 1;
        public static final double INTAKE_SENSOR_TO_MECH_RATIO = 1/1;
        public static final double INTAKE_MOTOR_SPEED = 60;
    }
}
