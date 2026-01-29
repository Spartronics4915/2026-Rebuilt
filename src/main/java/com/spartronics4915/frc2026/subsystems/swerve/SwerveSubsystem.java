package com.spartronics4915.frc2025.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.Drive.SwerveDirectories;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem {

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(SwerveDirectories swerveDir) {

        try {
            swerveDrive = new SwerveParse(new File(Filesystem.getDeployDirectory(), swerveDir.directory)).createSwerveDrive(SwerveConstants.kMaxSpeed.in(MetersPerSecond),
                guessStartingPosition()
            );

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setMotorIdleMode(true);
        swerveDrive.setChassisDiscretization(false, 0);
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAugularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 0);

        AutoBuilder.configure(
            this::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            (speeds, FF) -> {shimPublisher.accept(speeds); drive(speeds);},
            new PPHolomicDriveController(
                SwerveConstants.AutoConstants.kTranslationPID,
                SwerveConstants.AutoCOnstants.kRotationPID),
            SwerveConstants.AutoConstants.PathPlannerConfigs.COMP_CHASSIS.config,
            () -> {
                Optional<Alliance> temp = DriverStation.getAlliance();
                if(temp.isEmpty()) return false;
                if (temp.get() == Alliance.Red) {return true;}
                return false;
            }, this);
        
        SmartDashboard.putData("set angle to 0", Commands.runOnce(() -> {
            var currPose = getPose();
            setPose(new Pose2d(
                currPose.getX(),
                currPose.getY(),
                Rotation2d.kZero
            ));
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
    
    @Override
    public void onDisable() {
        swerveDrive.setMotorIdleMode(false);
    }

    @Override
    public void periodic() {
        posePublisher.accept(getPose());
    }
}