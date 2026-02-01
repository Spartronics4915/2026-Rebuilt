package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionConfiguration;
import com.spartronics4915.frc2026.subsystems.vision.results.ResultInterface;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ProcessorInterface {
    void process();
    List<ResultInterface> getResultQueue();
    
    void setPipeline(int pipelineIndex);
    String getCameraName();
    boolean isConnected();
    
    void start();
    void stop();

    void setRobotVelocitySupplier(Supplier<ChassisSpeeds> supplier);
    void setVisionConfiguration(VisionConfiguration configuration);

    default PhotonCamera getPhotonCamera() {return null;}
    default Transform3d getTransform() {return null;}
    default PhotonCameraSim getCameraSim() {return null;}
}
