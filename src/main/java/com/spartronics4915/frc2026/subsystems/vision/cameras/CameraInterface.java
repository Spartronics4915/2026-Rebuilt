package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

import com.spartronics4915.frc2026.subsystems.vision.configurations.CameraResult;
import com.spartronics4915.frc2026.subsystems.vision.configurations.VisionContext;

import edu.wpi.first.math.geometry.Transform3d;

public interface CameraInterface {
    List<CameraResult> processFrame(VisionContext context);
    List<CameraResult> getLatestResults();
    
    void setPipeline(int pipelineIndex);
    String getCameraName();
    boolean isConnected();
    
    void start();
    void stop();

    default PhotonCamera getPhotonCamera() {return null;}
    default Transform3d getTransform() {return null;}
    default PhotonCameraSim getCameraSim() {return null;}
}
