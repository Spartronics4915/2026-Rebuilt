package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.Optional;

import com.spartronics4915.frc2026.Constants.VisionConstants.CameraType;
import com.spartronics4915.frc2026.util.LimelightHelpers;

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

    //#region targetProcessing
    public Optional<Boolean> rawMajorityTargetSide(double[] rawTargets){
        int leftSideTargets = 0;
        int rightSideTargets = 0;
        boolean isLeftSideTarget = false;
        //group the data into sets containing 3 and get tnxc
        //should be mostly accurate
        for (int i = 0; i < rawTargets.length; i += 3){
            if (rawTargets[i] < 0){
                leftSideTargets++;
            } else{
                rightSideTargets++;
            }
        }
        if (rightSideTargets > leftSideTargets){
            return Optional.of(!isLeftSideTarget);
        } else if (leftSideTargets > rightSideTargets){
            return Optional.of(isLeftSideTarget);
        }
        return Optional.empty();
    }   
    public Optional<Boolean> bestTargetSide(){
        boolean isLeftSideTarget = false;
        double tx = LimelightHelpers.getTX(getName());
        if (tx < 0){
            return Optional.of(isLeftSideTarget);
        } else if (tx > 0){
            return Optional.of(!isLeftSideTarget);
        }
        return Optional.empty();
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

