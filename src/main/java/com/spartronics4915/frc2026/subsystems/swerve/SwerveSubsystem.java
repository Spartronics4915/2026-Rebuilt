package com.spartronics4915.frc2026.subsystems.swerve;

import java.io.File;
import java.io.IOException;

import com.spartronics4915.frc2026.Constants.SwerveConstants;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveDirectories;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public final class SwerveSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private final SwerveDrive swerveDrive;

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("logging").getStructTopic("pose", Pose2d.struct).publish();

    public SwerveSubsystem(SwerveDirectories swerveDir) {

        try {
            swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), swerveDir.directory)).createSwerveDrive(SwerveConstants.kMaxSpeed.in(SwerveConstants.MetersPerSecond),
             //new Pose2d(new Translation2d(meter.of(2),
             //    meter.of(5)),
             //    Rotation2d.fromDegrees(180)
             //),
                guessStartingPosition()
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setMotorIdleMode(true);
        swerveDrive.setChassisDiscretization(false, 0);
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 0);
        
        SmartDashboard.putData("set angle to 0", Commands.runOnce(() -> {
            var currPose = getPose();
            setPose(new Pose2d(
                currPose.getX(),
                currPose.getY(),
                Rotation2d.kZero
            ));
        }));

        SmartDashboard.putData(Commands.runOnce(() -> {
            setPose(new Pose2d(16.3,4, Rotation2d.fromDegrees(180.0)));
        }));

    }

    private static Pose2d guessStartingPosition() {

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
            return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        } else {
            return new Pose2d(0, 0, Rotation2d.fromDegrees(180.0));
        }
    }
 
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public void driveFieldOriented(ChassisSpeeds chassisSpeeds) {
        swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public void stopChassis() {
        drive(new ChassisSpeeds());
    }

    public SwerveDrive getInternalSwerve() {
        return swerveDrive;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    private void setPose(Pose2d pose2d) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPose'");
    }

    @Override
    public void periodic() {
        posePublisher.accept(getPose());
    }
}