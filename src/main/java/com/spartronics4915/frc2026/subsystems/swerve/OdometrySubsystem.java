package com.spartronics4915.frc2026.subsystems.swerve;

import java.util.ArrayList;
import java.util.Optional;

import com.spartronics4915.frc2026.Constants.OdometryConstants;
import com.spartronics4915.frc2026.Robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class OdometrySubsystem {

    //private final VisionDeviceSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    // private ArrayList<VisionMeasurment> visionMeasurements = new ArrayList<>();

    public OdometrySubsystem(/*VisionDeviceSubsystem visionSubsystem, */SwerveSubsystem swerveSubsystem) {
        //this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;
    }

    private void updateVisionMeasurements() {
        if (Robot.isReal()) {
            //visionMeasurements = ((LimelightVisionSubsystem) visionSubsystem).getVisionMeasurements();
        }
    }

    public Pose2d getPose() {
            return null;
    }
    
}