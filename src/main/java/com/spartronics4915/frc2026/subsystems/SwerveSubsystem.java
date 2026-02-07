package com.spartronics4915.frc2026.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.Robot;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;

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
    public static Pose2d movementOverride;
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
            driveController,
            config.pathplannerConfig.config,
            this::shouldFlip,
            this
        );
    }

    public boolean shouldFlip() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) return false;
        if (alliance.get() == Alliance.Red) {return true;}
        return false;
    }

    public Pose2d getRelativePose() {
        Pose2d pose = getPose();
        if (shouldFlip()) {
            return FlippingUtil.flipFieldPose(pose);
        } else {
            return pose;
        }
    }

    public RobotHeading getRelativeHeading() {
        RobotHeading heading = getHeading();
        if (shouldFlip()) {
            return new RobotHeading(
                new Rotation3d(
                    heading.rotation.getX(),
                    heading.rotation.getY() * -1,
                    heading.rotation.getZ() * -1
                ),
                heading.timestamp
            );
        } else {
            return heading;
        }
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

    public RobotHeading getHeading() {
        return new RobotHeading(swerveDrive.getGyroRotation3d(), Timer.getFPGATimestamp());
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public Pose2d getSimulatedPose() {
        return swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose();
    }

    public Pose2d getRobotPose() {
        if (Robot.isSimulation()) {
            return getSimulatedPose();
        } else {
            return getPose();
        }
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

    public Pose2d getMovementOverride() { 
        return movementOverride;
    }

    public void setMovementOverride(Pose2d override) {
        movementOverride = override;
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
            Pose2d currentPose = swerve.getPose();
            Pose2d override = swerve.getMovementOverride();

            // Raw joystick inputs
            double joyVX = applyResponseCurve(MathUtil.applyDeadband(driverController.getLeftY() * -1.0, STICK_DEADBAND)) * MAX_SPEED;
            double joyVY = applyResponseCurve(MathUtil.applyDeadband(driverController.getLeftX() * -1.0, STICK_DEADBAND)) * MAX_SPEED;
            double joyOmega = applyResponseCurve(MathUtil.applyDeadband(driverController.getRightX() * -1.0, STICK_DEADBAND)) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

            // Determine joystick components in field space
            ChassisSpeeds fieldJoy = ChassisSpeeds.fromRobotRelativeSpeeds(joyVX, joyVY, 0, isFieldRelative ? TELEOP_HEADING_OFFSET : currentPose.getRotation());
            double fieldVX = fieldJoy.vxMetersPerSecond;
            double fieldVY = fieldJoy.vyMetersPerSecond;

            if (override != null) {
                PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
                goalState.pose = override;

                ChassisSpeeds robotTarget = driveController.calculateRobotRelativeSpeeds(currentPose, goalState);
                ChassisSpeeds fieldTarget = ChassisSpeeds.fromRobotRelativeSpeeds(robotTarget, currentPose.getRotation());

                if (override.getX() != 0) fieldVX = fieldTarget.vxMetersPerSecond;
                if (override.getY() != 0) fieldVY = fieldTarget.vyMetersPerSecond;
                if (override.getRotation().getRadians() != 0) joyOmega = robotTarget.omegaRadiansPerSecond;
            }

            // Convert field-relative linear speeds back to robot-relative for the drivetrain
            return ChassisSpeeds.fromFieldRelativeSpeeds(fieldVX, fieldVY, joyOmega, currentPose.getRotation());
        };
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, SwerveSubsystem swerve) {
        return computeVelocitiesFromController(driverController, IS_FIELD_RELATIVE, swerve);
    }

    public static Supplier<ChassisSpeeds> getSwerveTeleopCSSupplier(XboxController driverController, SwerveSubsystem swerve){
        return computeVelocitiesFromController(driverController, swerve);
    }

    public Command driveCommand(ChassisSpeeds chassisSpeeds){
        return Commands.runOnce(() -> drive(chassisSpeeds));
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}
}
