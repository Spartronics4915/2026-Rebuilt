package com.spartronics4915.frc2026.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveDrive swerveDrive;
    public static Pose2d pose;
    private final File directory;
    
    public static boolean isRightAlliance;

    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();

    public SwerveSubsystem(SwerveConfigurations config) {
        this.directory = new File(Filesystem.getDeployDirectory(), config.directory);
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                MAX_SPEED,
                new Pose2d(new Translation2d(Meter.of(2), Meter.of(5)),
                Rotation2d.fromDegrees(120))
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        AutoBuilder.configure(
            this::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            (Speeds, FF) -> {drive(Speeds);},
            AutoConstants.kDriveController,
            config.pathplannerConfig.config,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if(alliance.isEmpty()) return false;
                if (alliance.get() == Alliance.Red) {return true;}
                return false;
            },
            this
        );
    }

    @Override
    public void periodic() {
        posePublisher.accept(getPose());
        setAllianceSide();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public RobotHeading getHeading() {
        return new RobotHeading(swerveDrive.getGyroRotation3d(), Timer.getFPGATimestamp());
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setAllianceSide() {
        if (pose.getX() >= 4.05) isRightAlliance = false;
            else isRightAlliance = true;
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

    public Command driveCommand(ChassisSpeeds chassisSpeeds){
        return Commands.runOnce(() -> drive(chassisSpeeds));
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}
}
