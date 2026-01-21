package com.spartronics4915.frc2026.subsystems.vision.cameras;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.spartronics4915.frc2026.Constants.VisionConstants.CameraType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class Luma implements Camera {
    final String name;
    final CameraType type;
    final int pipelineIndex;
    final AprilTagFieldLayout layout;
    Transform3d transform;
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;

    public Luma(String name, CameraType type, int pipelineIndex, AprilTagFieldLayout layout, Transform3d transform) {
        this.name = name;
        this.type = type;
        this.pipelineIndex = pipelineIndex;
        this.layout = layout;
        this.transform = transform;
        this.camera = new PhotonCamera(name);
        this.estimator = new PhotonPoseEstimator(layout, transform);
    }

    @Override public String getName() {return name;}
    @Override public CameraType getType() {return type;}
    @Override public int getPipelineIndex() {return pipelineIndex;}

    @Override public Optional<PhotonCamera> getCamera() {return Optional.of(camera);}
    @Override public Optional<PhotonPoseEstimator> getEstimator() {return Optional.of(estimator);}
    @Override public Optional<AprilTagFieldLayout> getLayout() {return Optional.of(layout);}
    @Override public Optional<Transform3d> getTransform() {return Optional.of(transform);}
}
