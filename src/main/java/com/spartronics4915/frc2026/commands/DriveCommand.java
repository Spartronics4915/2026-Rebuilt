package com.spartronics4915.frc2026.commands;

import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Default teleop drive command.
 *
 * <p>This command intentionally contains no drivetrain policy. It only selects the
 * active controller and forwards raw joystick values to the drivetrain.</p>
 */
public class DriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CommandXboxController driverController;
    private final CommandXboxController testingController;

    public DriveCommand(
        CommandXboxController driverController,
        CommandXboxController testingController,
        SwerveSubsystem swerve
    ) {
        this.swerve = swerve;
        this.driverController = driverController;
        this.testingController = testingController;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        XboxController controller = resolveController();

        swerve.acceptTeleopInput(
            -controller.getLeftY(),
            -controller.getLeftX(),
            -controller.getRightX()
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    private XboxController resolveController() {
        if (testingController != null) {
            XboxController controller = testingController.getHID();
            if (controller.isConnected()) {
                return controller;
            }
        }

        return driverController.getHID();
    }
}