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
        public static final int INTAKE_MOTOR_ID = 0;

        public static final int INTAKE_P = 0;
        public static final int INTAKE_I = 0;
        public static final int INTAKE_D = 0;

        public static final boolean INTAKE_CURRENT_LIMIT_ENABLE = true;
        public static final int INTAKE_CURRENT_LIMIT = 40;
        public static final int INTAKE_CURRENT_LOWER_LIMIT = 20;
        public static final int INTAKE_CURRENT_LOWER_TIME = 0;
        public static final int INTAKE_SENSOR_TO_MECH_RATIO = 1/1;

        public static final double INTAKE_MOTOR_SPEED = 1.0;
    }
}
