package com.spartronics4915.frc2025.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.spartronics4915.frc2025.Robot;
import com.spartronics4915.frc2025.Constants.OdometryConstants;
import com.spartronics4915.frc2025.subsystems.vision.LimelightVisionSubsystem;
import com.spartronics4915.frc2025.subsystems.vision.VisionDeviceSubystem;
import com.spartronics4915.frc2025.util.Structures.VisionMeasurement;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem {

    private final VisionDeviceSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    private ArrayList<VisionMeasurment> visionMeasurements = new ArrayList<>();

    public OdometrySubsystem(VisionDeviceSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
    }

    private void updateVisionMeasurements() {
        if (Robot.isReal()) {
            visionMeasurements = ((LimelightVisionSubsystem) visionSubsystem).getVisionMeasurements();
        }
    }

    private Optional<Pose2d> getVisionPose() {
        if (Robot.isSimulation()) {
            return visionSubsystem.getBotPose2dFromCamera();
        }
        if (visionMeasurements.size() == 0) return Optional.empty();
        if (visionMeasurements.size() == 1) return Optional.of(visionMeasurements.get(0).pose());
        return Optional.of(
            visioMeasurements.get(0).pose().interpolate(
                visionMeasurements.get(1).pose(),
                0)
        );
    }

    public Pose2d getPose() {
        Pose2d swervePose = swerveSubsystem.getPose();
        Optional<Pose2d> potentialVisionPose = getVisionPose();
        if (potentialVisionPose.isEmpty()) return swervePose;
        Pose2d visionPose = potentialVisionPose.get();
        double distance = swervePose.getTranslation().getDistance(visionPose.getTranslation());
        if (distance > OdometryConstants.kMaxSwerveVisionPoseDifference) return visionPose;
        return swervePose;
    }

    @Override
    public void periodic() {

    }
    
}
