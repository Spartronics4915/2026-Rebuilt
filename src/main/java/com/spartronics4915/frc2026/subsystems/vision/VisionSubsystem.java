package com.spartronics4915.frc2026.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class VisionSubsystem extends SubsystemBase {
    private final CameraSource[] sources;
    private final VisionConsumer consumer;
    private final Supplier<Rotation2d> robotYawSupplier;

    public VisionSubsystem(Supplier<Rotation2d> robotYawSupplier, VisionConsumer consumer, CameraSource... sources) {
        this.robotYawSupplier = robotYawSupplier;
        this.consumer = consumer;
        this.sources = sources;
    }

    @Override
    public void periodic() {
        Rotation2d currentYaw = robotYawSupplier.get();

        for (CameraSource source : sources) {
            // 1. Sync the camera with current robot state
            source.updateHeading(currentYaw);

            // 2. Process all observations from this camera
            for (var observation : source.getObservations()) {
                if (isPoseValid(observation)) {
                    
                    // 3. Scale confidence (Std Devs) based on distance and tag count
                    double stdDevFactor = Math.pow(observation.averageDistance(), 3.0) / observation.tagCount();
                    double linStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
                    double angStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                    // 4. Feed to Pose Estimator (Drivetrain)
                    consumer.accept(
                        observation.robotPose(), 
                        observation.timestamponds(), 
                        VecBuilder.fill(linStdDev, linStdDev, angStdDev)
                    );
                }
            }
        }
    }

    private boolean isPoseValid(FiducialObservation.VisionObservation obs) {
        // Field boundary and ambiguity checks (as seen in your Constants)
        if (obs.tagCount() == 1 && obs.ambiguity() > 0.2) return false;
        if (obs.robotPose().getX() < 0 || obs.robotPose().getX() > 16.5) return false;
        return true;
    }

}