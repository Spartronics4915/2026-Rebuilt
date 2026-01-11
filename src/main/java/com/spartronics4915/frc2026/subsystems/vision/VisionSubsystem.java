package com.spartronics4915.frc2026.subsystems.vision;

import static com.spartronics4915.frc2026.Constants.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

import com.spartronics4915.frc2026.Constants.VisionConstants.LimelightConfiguration;
import com.spartronics4915.frc2026.util.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private List<String> limelights = new ArrayList<>();

    public VisionSubsystem() {
        for (LimelightConfiguration configuration : limelightConfigurations) {
            limelights.add(configuration.name());
            LimelightHelpers.setCameraPose_RobotSpace(
                configuration.name(),
                configuration.robotToCamera().get(0),
                configuration.robotToCamera().get(1),
                configuration.robotToCamera().get(2),
                configuration.robotToCamera().get(3),
                configuration.robotToCamera().get(4),
                configuration.robotToCamera().get(5)
            );
            limelights.add(configuration.name());
            System.out.println(configuration.name() + " Successfully Configured");
        }
    }

    @Override
    public void periodic() {

    }
}
