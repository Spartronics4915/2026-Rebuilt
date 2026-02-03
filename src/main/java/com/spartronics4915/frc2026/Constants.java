// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class ShooterConstants {
        
        public static double deltaTime = 0;
        
        //Main motor----------------------------------
        public static int mainShooterMotorID = 100;
        // Pid control of mainShooterMotor
        public static double MainP = 0;
        public static double MainI = 0;
        public static double MainD = 0;
        
        //Follower motor-------------------------------
        public static int followerShooterMotorID = 99;
        // Pid control 
        public static double FollowerP = 0;
        public static double FollowerI = 0;
        public static double FollowerD = 0;

        //both motors----------------------------------
        public static boolean   SupplyCurrentLimitEnabled   = true;
        public static double    SupplyCurrentLimit          = 60;
        public static double    SupplyCurrentLowerLimit     = 40;
        public static double    SupplyCurrentLowerTime      = 1.0;
        public static double    SensorToMechanismRatio      = 1;
        
        
        
        
        //feed forward
        public static double S = 0;
        public static double V = 0;
        public static double A = 0;

        //trap profile
        public static double MaxVelocity = 20;     //is accel
        public static double MaxAcceleration = 20; //is jerk

        public static boolean motorTurnsClockWise = true;
        //Main motor, looking at face of motor, true for clock wise , false for counter clock wise
        public static boolean motorCoast = true;
        //true for Coast, false for Brake

        //is the min and max speed of the motors
        public static double minSpeed = 0;
        public static double maxSpeed = 40;

        



    }
}
