package com.spartronics4915.frc2026.commands;

import static com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem.*;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<ChassisSpeeds> speedSupplier;

    public DriveCommand(CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.speedSupplier = getSwerveTeleopCSSupplier(driverController.getHID(), swerveSubsystem);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = speedSupplier.get();
        swerveSubsystem.drive(chassisSpeeds);
    }
}
