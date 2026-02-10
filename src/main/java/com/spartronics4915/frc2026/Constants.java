// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class ShooterConstants {
        
        public static final int LEAD_MOTOR_ID = 22;
        public static final int FOLLOWER_MOTOR_ID = 23;

        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0.005;

        public static final SlotConfigs PID_CONFIG = new SlotConfigs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLowerTime(1.0);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(0.9375)
        ;

    }
}
