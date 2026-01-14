package com.spartronics4915.frc2026.subsystems.vision;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Camera;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Limelight;
import com.spartronics4915.frc2026.subsystems.vision.cameras.Luma;
import com.spartronics4915.frc2026.util.LimelightHelpers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private static final HashMap<String, Camera> cameras = new HashMap<>();

    private Luma luma;
    private Limelight limelight;

    private boolean isSimulation;
    private VisionSystemSim photonSim;
    private PhotonCameraSim lumaSim;
     
    public VisionSubsystem() {
        isSimulation = Robot.isSimulation();
        if (isSimulation) {
            photonSim = new VisionSystemSim("photon");
            photonSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
            PhotonCamera.setVersionCheckEnabled(false);
        }
        
        // TODO: Possibly add a check if the cameras are on and connected, so we could handle the issues ourselves without the robot code crashing
        for (Camera camera : cameraList) {
            switch (camera.getType()) {
                case LUMA:
                    luma = new Luma(
                        camera.getName(), 
                        camera.getType(), 
                        camera.getLayout().get(), 
                        camera.getTransform().get()
                    );
                    cameras.put(getName(), luma);
                    
                    if (isSimulation) {
                        lumaSim = new PhotonCameraSim(luma.getCamera().get(), simCameraProperties);
                        photonSim.addCamera(lumaSim, luma.getTransform().get());
                    }
                    break;
            
                case LIMELIGHT:
                    limelight = new Limelight(
                        camera.getName(), 
                        camera.getType(),
                        camera.getX().get(), 
                        camera.getY().get(), 
                        camera.getZ().get(), 
                        camera.getYaw().get(), 
                        camera.getPitch().get(), 
                        camera.getRoll().get()
                    );
                    LimelightHelpers.setCameraPose_RobotSpace(
                        limelight.getName(),
                        limelight.getX().orElse(0.0), 
                        limelight.getY().orElse(0.0), 
                        limelight.getZ().orElse(0.0), 
                        limelight.getYaw().orElse(0.0), 
                        limelight.getPitch().orElse(0.0), 
                        limelight.getRoll().orElse(0.0)
                    );
                    cameras.put(limelight.getName(), limelight);
                    break;
            }
        }
    }

    @Override
    public void periodic() {
        /* for (Camera camera : cameras) 
            switch (camera.type) {
                case LUMA:
                    switch (state) {
                        if (!camera.hasTargets()) continue;
                            case GLOBAL:

                            break;

                            case LOCAL:

                            break;
                    }
                    break;
            
                case LIMELIGHT:
                    switch (camera.pipeline) {
                        case 0:

                            break;
                        
                        case 1:

                            break;
                    }
                    break;
            }
        */
    }
}
