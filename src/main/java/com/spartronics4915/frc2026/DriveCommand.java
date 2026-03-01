package com.spartronics4915.frc2026;

import java.util.function.Supplier;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<ChassisSpeeds> speedSupplier;

    public DriveCommand(
        CommandXboxController driverController,
        CommandXboxController testingController,
        SwerveSubsystem swerveSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;

        Supplier<XboxController> controllerSupplier = (testingController != null)
            ? () -> {
                XboxController hid = testingController.getHID();
                return hid.isConnected() ? hid : driverController.getHID();
              }
            : driverController::getHID;

        this.speedSupplier = SwerveSubsystem.getSwerveTeleopCSSupplier(
            controllerSupplier, 
            swerveSubsystem
        );

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(speedSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}