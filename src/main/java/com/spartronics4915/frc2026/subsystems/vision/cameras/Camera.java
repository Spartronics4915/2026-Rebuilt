package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.spartronics4915.frc2026.Constants.VisionConstants.CameraType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public interface Camera {
    String getName();
    CameraType getType();
    
    default Optional<Double> getX() {return Optional.empty();}
    default Optional<Double> getY() {return Optional.empty();}
    default Optional<Double> getZ() {return Optional.empty();}
    default Optional<Double> getYaw() {return Optional.empty();}
    default Optional<Double> getRoll() {return Optional.empty();}
    default Optional<Double> getPitch() {return Optional.empty();}

    default Optional<PhotonCamera> getCamera() {return Optional.empty();}
    default Optional<PhotonPoseEstimator> getEstimator() {return Optional.empty();}
    default Optional<AprilTagFieldLayout> getLayout() {return Optional.empty();}
    default Optional<Transform3d> getTransform() {return Optional.empty();}
}
