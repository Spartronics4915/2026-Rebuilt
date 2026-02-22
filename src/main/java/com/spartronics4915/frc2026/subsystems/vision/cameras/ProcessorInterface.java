package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;

public interface ProcessorInterface {

    // ------ Main Functionality ------
    
    void start();
    void stop();
    void process();

    // ------ Getters ------

    String getCameraName();
    PhotonCamera getPhotonCamera();
    AprilTagFieldLayout getFieldlayout();
    Transform3d getCameraTransform();
    PhotonPoseEstimator getPoseEstimator();
    SimCameraProperties getSimProperties();
    PhotonCameraSim getCameraSim();
    List<ResultInterface> getResultQueue();
    int getMaxQueueSize();
    Notifier getNotifier();
    double getFrequency();
    boolean isRunning();
    Supplier<ChassisSpeeds> getSpeedSupplier();
    
    // ------ Setters ------

    void setPipeline(int newPipelineIndex);
    void setCameraTransform(Transform3d newCameraTransform);
    void setSpeedSupplier(Supplier<ChassisSpeeds> speedSupplier);
    
}
