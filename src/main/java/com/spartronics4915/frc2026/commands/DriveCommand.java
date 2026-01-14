package com.spartronics4915.frc2026.commands;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import static com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;

    public DriveCommand(CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = getSwerveTeleopCSSupplier(driverController.getHID(), swerveSubsystem).get();
        swerveSubsystem.drive(chassisSpeeds);
    }
}
