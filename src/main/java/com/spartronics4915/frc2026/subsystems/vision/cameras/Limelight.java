package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.Optional;

import com.spartronics4915.frc2026.Constants.VisionConstants.CameraType;


public class Limelight implements Camera {
    final String name;
    final CameraType type;
    final double x;
    final double y;
    final double z;
    final double yaw;
    final double pitch;
    final double roll;

    public Limelight(
        String name, 
        CameraType type, 
        double x, 
        double y, 
        double z, 
        double yaw, 
        double pitch, 
        double roll
    ) { 
        this.name = name; 
        this.type = type;
        this.x = x;
        this.y = y;
        this.z = z;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
    }

    @Override public String getName() {return name;}
    @Override public CameraType getType() {return type;}
    @Override public Optional<Double> getX() {return Optional.of(x);}
    @Override public Optional<Double> getY() {return Optional.of(y);}
    @Override public Optional<Double> getZ() {return Optional.of(z);}
    @Override public Optional<Double> getYaw() {return Optional.of(yaw);}
    @Override public Optional<Double> getRoll() {return Optional.of(roll);}
    @Override public Optional<Double> getPitch() {return Optional.of(pitch);}
}

