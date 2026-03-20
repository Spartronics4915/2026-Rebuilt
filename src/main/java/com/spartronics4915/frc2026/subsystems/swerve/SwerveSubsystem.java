package com.spartronics4915.frc2026.subsystems.swerve;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.util.swerve.SlipDetector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrivetrain<?, ?, ?> drivetrain;
    private final SwerveConfigurations activeConfig;

    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularSpeed.baseUnitMagnitude() * 0.1);

    private final SwerveRequest.FieldCentricFacingAngle headingLockRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(maxSpeed * 0.1);

    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularSpeed.baseUnitMagnitude() * 0.1);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.SwerveDriveBrake lockRequest =
        new SwerveRequest.SwerveDriveBrake();

    private final SlipDetector slipDetector = new SlipDetector();
    private final AtomicBoolean currentlySlipping = new AtomicBoolean(false);
    private boolean isInSlipRecovery = false;
    private double lastSlipTimestamp = -1.0;

    private Pose3d pose3d = new Pose3d();
    private double movementOverride = 0.0;
    private boolean isFieldRelativeState = defaultFieldRelative;
    private Rotation2d teleopHeadingOffset = Rotation2d.kZero;

    private Optional<Alliance> cachedAlliance = Optional.empty();
    private boolean hasCheckedAlliance = false;

    private final StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> pose3dPublisher =
        NetworkTableInstance.getDefault().getStructTopic("Pose3d", Pose3d.struct).publish();

    public SwerveSubsystem(SwerveConfigurations config) {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            config.drivetrainConstants,
            odomUpdateFrequency,
            config.modules[0], config.modules[1], config.modules[2], config.modules[3]
        );
        activeConfig = config;

        drivetrain.setStateStdDevs(normalStdDevs);

        headingLockRequest.HeadingController.setPID(headingLockKP, 0, headingLockKD);
        headingLockRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        if (Utils.isSimulation()) {
            drivetrain.resetPose(
                new Pose2d(new Translation2d(2.0, 5.0), Rotation2d.fromDegrees(120))
            );
        }

        drivetrain.registerTelemetry(this::updateOdometry);
        configurePathPlanner();
    }

    private void updateOdometry(SwerveDriveState state) {
        currentlySlipping.set(slipDetector.update(state.ModuleStates, state.ModuleTargets));
    }

    @Override
    public void periodic() {
        double now = Utils.getCurrentTimeSeconds();

        if (currentlySlipping.get()) {
            lastSlipTimestamp = now;
            if (!isInSlipRecovery) {
                isInSlipRecovery = true;
                drivetrain.setStateStdDevs(slipStdDevs);
            }
        } else if (isInSlipRecovery && (now - lastSlipTimestamp) > slipRecoverySeconds) {
            isInSlipRecovery = false;
            drivetrain.setStateStdDevs(normalStdDevs);
        }

        Pose2d pose = getPose();
        posePublisher.set(pose);
        pose3d = new Pose3d(
            pose.getX(), pose.getY(), 0,
            new Rotation3d(getRoll().getRadians(), getPitch().getRadians(),
                pose.getRotation().getRadians())
        );
        pose3dPublisher.set(pose3d);
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    public void driveFieldCentric(double vX, double vY, double omega) {
        drivetrain.setControl(
            fieldCentricRequest
                .withVelocityX(vX)
                .withVelocityY(vY)
                .withRotationalRate(omega)
        );
    }

    public void driveFieldCentricFacingAngle(double vX, double vY, Rotation2d targetHeading) {
        drivetrain.setControl(
            headingLockRequest
                .withVelocityX(vX)
                .withVelocityY(vY)
                .withTargetDirection(targetHeading)
        );
    }

    /** Robot-centric drive (used when field-relative is toggled off). */
    public void driveRobotCentric(double vX, double vY, double omega) {
        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(vX)
                .withVelocityY(vY)
                .withRotationalRate(omega)
        );
    }

    public void setDriverPerspective(Rotation2d heading) {
        drivetrain.setOperatorPerspectiveForward(heading);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drivetrain.setControl(autoRequest.withSpeeds(chassisSpeeds));
    }

    public void lockModules() {
        drivetrain.setControl(lockRequest);
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    /** Pose flipped to the current alliance's perspective. Use for all game logic. */
    public Pose2d getRelativePose() {
        Pose2d pose = getPose();
        return shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public Pose2d getPastVisionPose(double timestamp) {
        try {
            return drivetrain.samplePoseAt(timestamp).orElse(getPose());
        } catch (Exception e) {
            System.err.println("Warning: Could not sample pose at timestamp " + timestamp);
            return getPose();
        }
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        drivetrain.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return drivetrain.getState().Speeds;
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getPose().getRotation());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        ChassisSpeeds v = getFieldVelocity();
        return shouldFlip() ? FlippingUtil.flipFieldSpeeds(v) : v;
    }

    public double getSpeed() {
        ChassisSpeeds v = getFieldVelocity();
        return Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(drivetrain.getPigeon2().getRoll().getValueAsDouble());
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(drivetrain.getPigeon2().getPitch().getValueAsDouble());
    }

    public Rotation3d getGyroRotation3d() {
        return new Rotation3d(
            getRoll().getRadians(),
            getPitch().getRadians(),
            getPose().getRotation().getRadians()
        );
    }

    public RobotHeading getHeading() {
        return new RobotHeading(getGyroRotation3d(), Timer.getFPGATimestamp());
    }

    public boolean isFlat() {
        return Math.abs(getPitch().getDegrees()) < tiltThresholdDegrees
            && Math.abs(getRoll().getDegrees()) < tiltThresholdDegrees;
    }

    private final Trigger flatTrigger = new Trigger(this::isFlat).debounce(tiltDebounce);

    public boolean isFlatDebounced() {
        return flatTrigger.getAsBoolean();
    }

    public boolean isFieldRelative() {
        return isFieldRelativeState;
    }

    public void setFieldRelative(boolean fieldRelative) {
        isFieldRelativeState = fieldRelative;
    }

    public void toggleFieldRelative() {
        isFieldRelativeState = !isFieldRelativeState;
    }

    public Rotation2d getHeadingOffset() {
        return teleopHeadingOffset;
    }

    /** Snaps the driver's forward perspective to the robot's current heading. */
    public void resetHeadingOffset() {
        teleopHeadingOffset = getPose().getRotation();
    }

    public double getMovementOverride() {
        return movementOverride;
    }

    public void setMovementOverride(double override) {
        movementOverride = override;
    }

    public boolean isSlipping() {
        return isInSlipRecovery;
    }

    public boolean shouldFlip() {
        if (!hasCheckedAlliance) {
            cachedAlliance = DriverStation.getAlliance();
            if (cachedAlliance.isPresent()) {
                hasCheckedAlliance = true;
            }
        }
        return cachedAlliance.isPresent() && cachedAlliance.get() == Alliance.Red;
    }

    public void resetAlliance() {
        cachedAlliance = Optional.empty();
        hasCheckedAlliance = false;
    }

    private void configurePathPlanner() {
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotVelocity,
            (speeds, ff) -> drive(speeds),
            driveController,
            activeConfig.pathplannerConfig.config,
            this::shouldFlip,
            this
        );
    }

    public static ChassisSpeeds rotateLinearChassisSpeeds(ChassisSpeeds in, Rotation2d offset) {
        Translation2d linear = new Translation2d(
            in.vxMetersPerSecond, in.vyMetersPerSecond).rotateBy(offset);
        return new ChassisSpeeds(linear.getX(), linear.getY(), in.omegaRadiansPerSecond);
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}

}