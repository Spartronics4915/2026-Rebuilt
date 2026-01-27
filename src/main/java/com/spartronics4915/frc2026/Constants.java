// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
    public static class ClimberConstants{
        public static final int PRIMARY_CLIMB_MOTOR_ID = 0;
        public static final double SENSOR_TO_MECHANISM_RATIO = 0.0;
        public static final double CLIMBER_P = 0.0;
        public static final double CLIMBER_I = 0.0;
        public static final double CLIMBER_D = 0.0;
        public static final double MAX_VELOCITY = 0.0;
        public static final double MAX_ACCELERATION = 0.0;

        public static final double CLIMBER_S = 0;
        public static final double CLIMBER_J = 0;
        public static final double CLIMBER_V = 0;
        public static final double CLIMBER_A = 0;

        public static final boolean CURRENT_LIMIT_ENABLED = true;
        public static final double SUPPLY_CURRENT_LIMIT = 25;
        public static final double CURRENT_LOWER_LIMIT = 10;
        public static final double CURRENT_LOWER_TIME = 1.0;

        public static final double MIN_HIGHT = 0.0;
        public static final double MAX_HIGHT = 1.0;
        public static final double DeltaTime = 1.0/50.0;


    }
}
