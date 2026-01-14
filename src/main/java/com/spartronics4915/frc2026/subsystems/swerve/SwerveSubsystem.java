package com.spartronics4915.frc2026.subsystems.swerve;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import static com.spartronics4915.frc2026.Constants.IO.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveDrive swerveDrive;
    public static Pose2d swervePose;
    private final File directory = new File(Filesystem.getDeployDirectory(), "swerve/test-chassis");

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();

    public SwerveSubsystem() {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                MAX_SPEED,
                new Pose2d(new Translation2d(Meter.of(5), Meter.of(3)),
                Rotation2d.fromDegrees(0))
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    @Override
    public void periodic() {
        posePublisher.accept(getPose());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public Rotation2d getYaw() {
        return swerveDrive.getYaw();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Pose2d getPastVisionPose(double timestamp) {
        return swerveDrive.swerveDrivePoseEstimator.sampleAt(timestamp).get();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public double getSpeed() {
        ChassisSpeeds fieldVelocity = getFieldVelocity();
        return Math.sqrt(fieldVelocity.vxMetersPerSecond * fieldVelocity.vxMetersPerSecond + fieldVelocity.vyMetersPerSecond * fieldVelocity.vyMetersPerSecond);
    }

    static private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    public static ChassisSpeeds rotateLinearChassisSpeeds(ChassisSpeeds in, Rotation2d offset){
        Translation2d modifiedLinear = new Translation2d(
            in.vxMetersPerSecond,
            in.vyMetersPerSecond
        ).rotateBy(offset);

        return new ChassisSpeeds(
            modifiedLinear.getX(),
            modifiedLinear.getY(), 
            in.omegaRadiansPerSecond
        );
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, boolean isFieldRelative, SwerveSubsystem swerve) {
        return () -> {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    
            final double inputXRaw = driverController.getLeftY() * -1.0;
            final double inputYRaw = driverController.getLeftX() * -1.0;
            final double inputOmegaRaw;
            
            inputOmegaRaw = driverController.getRightX() * -1.0;
    
            final double inputX = applyResponseCurve(MathUtil.applyDeadband(inputXRaw, STICK_DEADBAND));
            final double inputY = applyResponseCurve(MathUtil.applyDeadband(inputYRaw, STICK_DEADBAND));
            final double inputOmega = applyResponseCurve(MathUtil.applyDeadband(inputOmegaRaw, STICK_DEADBAND));
    
            chassisSpeeds.vxMetersPerSecond = inputX * MAX_SPEED;
            chassisSpeeds.vyMetersPerSecond = inputY * MAX_SPEED;
            chassisSpeeds.omegaRadiansPerSecond = inputOmega * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

            if (isFieldRelative) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, swerve.getPose().getRotation());
            }
            
            chassisSpeeds = rotateLinearChassisSpeeds(chassisSpeeds, TELEOP_HEADING_OFFSET);
    
            return chassisSpeeds;
        };
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, SwerveSubsystem swerve) {
        return computeVelocitiesFromController(driverController, IS_FIELD_RELATIVE, swerve);
    }

    public static Supplier<ChassisSpeeds> getSwerveTeleopCSSupplier(XboxController driverController, SwerveSubsystem swerve){
        return () -> {
            ChassisSpeeds chassisSpeeds = computeVelocitiesFromController(driverController, swerve).get();
            return chassisSpeeds;
        };
    }
}
