package com.spartronics4915.frc2026.subsystems.swerve;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.util.general.MovingAveragePose;
import com.spartronics4915.frc2026.util.swerve.SlipDetector;
import com.spartronics4915.frc2026.util.vision.ConcurrentTimeBuffer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrivetrain<?, ?, ?> drivetrain;
    private final SwerveConfigurations activeConfig;

    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.FieldCentricFacingAngle headingLockRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.RobotCentric robotCentricRequest =
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest =
        new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.SwerveDriveBrake lockRequest =
        new SwerveRequest.SwerveDriveBrake();

    private final SlipDetector slipDetector = new SlipDetector();
    private final AtomicBoolean currentlySlipping = new AtomicBoolean(false);
    private boolean isInSlipRecovery = false;
    private double lastSlipTimestamp = -1.0;

    private final ConcurrentTimeBuffer<Double> yawRateBuffer = ConcurrentTimeBuffer.createDoubleBuffer(1.0);

    private double prevYawRad = Double.NaN;
    private double prevYawTimestamp = Double.NaN;

    private Pose2d smoothedPose = new Pose2d();

    private final MovingAveragePose poseFilter =
        new MovingAveragePose(0.30); // previously 0.20

    private double movementOverride = 0.0;
    private boolean isFieldRelativeState = defaultFieldRelative;
    private Rotation2d teleopHeadingOffset = Rotation2d.kZero;

    private Optional<Alliance> cachedAlliance = Optional.empty();
    private boolean hasCheckedAlliance = false;

    private final Debouncer flatDebouncer = new Debouncer(tiltDebounce);
    private boolean isFlatDebouncedValue = false;

    private final SwerveTelemetry telemetry = new SwerveTelemetry();

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
                new Pose2d(new Translation2d(14.0, 5.0), Rotation2d.fromDegrees(0))
            );
        }

        drivetrain.registerTelemetry(this::updateOdometry);
        configurePathPlanner();
    }

    private void updateOdometry(SwerveDriveState state) {
        currentlySlipping.set(slipDetector.update(state.ModuleStates, state.ModuleTargets));

        // Record yaw rate numerically from consecutive odometry poses
        double nowYaw = state.Pose.getRotation().getRadians();
        double nowTs = state.Timestamp;
        if (!Double.isNaN(prevYawTimestamp)) {
            double dt = nowTs - prevYawTimestamp;
            if (dt > 0) {
                double yawRate = MathUtil.angleModulus(nowYaw - prevYawRad) / dt;
                yawRateBuffer.addSample(nowTs, yawRate);
            }
        }
        prevYawRad = nowYaw;
        prevYawTimestamp = nowTs;
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

        isFlatDebouncedValue = flatDebouncer.calculate(isFlat());

        smoothedPose = poseFilter.calculate(getRelativePose());
 
        telemetry.publish(drivetrain.getState(), this);
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
        poseFilter.reset(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return drivetrain.getState().Speeds;
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getPose().getRotation());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        ChassisSpeeds velocity = getFieldVelocity();
        return shouldFlip() ? 
            FlippingUtil.flipFieldSpeeds(velocity) 
            : velocity;
    }

    public double getSpeed() {
        ChassisSpeeds velocity = getFieldVelocity();
        return Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(
            drivetrain.getPigeon2().getRoll().getValueAsDouble()
        );
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(
            drivetrain.getPigeon2().getPitch().getValueAsDouble()
        );
    }

    public Rotation3d getGyroRotation3d() {
        return drivetrain.getRotation3d();
    }

    public RobotHeading getHeading() {
        return new RobotHeading(getGyroRotation3d(), Timer.getFPGATimestamp());
    }

    public Pose2d getSmoothedPose() {
        return smoothedPose;
    }

    public OptionalDouble getMaxAbsYawRateInRange(double minTime, double maxTime) {
        return yawRateBuffer.getMaxAbsValueInRange(minTime, maxTime);
    }

    public boolean isFlat() {
        return Math.abs(getPitch().getDegrees()) < tiltThresholdDegrees
            && Math.abs(getRoll().getDegrees()) < tiltThresholdDegrees;
    }

    public boolean isFlatDebounced() {
        return isFlatDebouncedValue;
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
            in.vxMetersPerSecond, 
            in.vyMetersPerSecond
        ).rotateBy(offset);
        return new ChassisSpeeds(linear.getX(), linear.getY(), in.omegaRadiansPerSecond);
    }

    public void configureStandardDevsForDisabled() {
        drivetrain.setStateStdDevs(VecBuilder.fill(1.0, 1.0, 1.0));
    }

    public void configureStandardDevsForEnabled() {
        drivetrain.setStateStdDevs(normalStdDevs);
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}

    private final class SwerveTelemetry {
 
        private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
 
        private final NetworkTable root = NetworkTableInstance.getDefault().getTable("swerve");

        private final StructPublisher<Pose2d> pose = root.getStructTopic("Pose", Pose2d.struct).publish();
        private final StructPublisher<Pose2d> smoothed = root.getStructTopic("SmoothedPose", Pose2d.struct).publish();
        private final StructPublisher<ChassisSpeeds> measuredSpeeds = root.getStructTopic("MeasuredSpeeds", ChassisSpeeds.struct).publish();
        private final StructPublisher<ChassisSpeeds> fieldSpeeds = root.getStructTopic("FieldRelativeSpeeds", ChassisSpeeds.struct).publish();
 
        private final StructArrayPublisher<SwerveModuleState> moduleStates = root.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
        private final StructArrayPublisher<SwerveModuleState> moduleTargets = root.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
        private final StructArrayPublisher<SwerveModulePosition> modulePositions = root.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
 
        private final DoublePublisher speed = root.getDoubleTopic("SpeedMPS").publish();
        private final DoublePublisher odometryHz = root.getDoubleTopic("OdometryHz").publish();
        private final DoublePublisher batteryV = root.getDoubleTopic("BatteryVoltage").publish();
        private final DoublePublisher headingDeg = root.getDoubleTopic("HeadingDeg").publish();
        private final DoublePublisher rollDeg = root.getDoubleTopic("RollDeg").publish();
        private final DoublePublisher pitchDeg = root.getDoubleTopic("PitchDeg").publish();
        private final DoublePublisher yawRateRadS = root.getDoubleTopic("YawRateRadS").publish();
        private final BooleanPublisher slipping = root.getBooleanTopic("IsSlipping").publish();
        private final BooleanPublisher slipRecovery = root.getBooleanTopic("IsInSlipRecovery").publish();
        private final BooleanPublisher fieldRelative = root.getBooleanTopic("IsFieldRelative").publish();
 
        private final DoublePublisher[] driveVelMPS = new DoublePublisher[4];
        private final DoublePublisher[] driveTargetMPS = new DoublePublisher[4];
        private final DoublePublisher[] drivePositionM = new DoublePublisher[4];
        private final DoublePublisher[] driveCurrentA = new DoublePublisher[4];
        private final DoublePublisher[] driveVoltageV = new DoublePublisher[4];
        private final DoublePublisher[] driveTempC = new DoublePublisher[4];
        private final DoublePublisher[] driveCLError = new DoublePublisher[4];
        private final DoublePublisher[] driveClosedLoopRef = new DoublePublisher[4];
 
        private final DoublePublisher[] steerAngleDeg = new DoublePublisher[4];
        private final DoublePublisher[] steerTargetDeg = new DoublePublisher[4];
        private final DoublePublisher[] steerCurrentA = new DoublePublisher[4];
        private final DoublePublisher[] steerVoltageV = new DoublePublisher[4];
        private final DoublePublisher[] steerTempC = new DoublePublisher[4];
        private final DoublePublisher[] steerCLError = new DoublePublisher[4];
 
 
        SwerveTelemetry() {
            for (int i = 0; i < 4; i++) {
                NetworkTable t = root.getSubTable("modules/" + MODULE_NAMES[i]);
 
                driveVelMPS[i] = t.getDoubleTopic("DriveVelocityMPS").publish();
                driveTargetMPS[i] = t.getDoubleTopic("DriveTargetMPS").publish();
                drivePositionM[i] = t.getDoubleTopic("DrivePositionM").publish();
                driveCurrentA[i] = t.getDoubleTopic("DriveCurrentA").publish();
                driveVoltageV[i] = t.getDoubleTopic("DriveVoltageV").publish();
                driveTempC[i] = t.getDoubleTopic("DriveTempC").publish();
                driveCLError[i] = t.getDoubleTopic("DriveClosedLoopError").publish();
                driveClosedLoopRef[i] = t.getDoubleTopic("DriveClosedLoopRef").publish();
 
                steerAngleDeg[i] = t.getDoubleTopic("SteerAngleDeg").publish();
                steerTargetDeg[i] = t.getDoubleTopic("SteerTargetDeg").publish();
                steerCurrentA[i] = t.getDoubleTopic("SteerCurrentA").publish();
                steerVoltageV[i] = t.getDoubleTopic("SteerVoltageV").publish();
                steerTempC[i] = t.getDoubleTopic("SteerTempC").publish();
                steerCLError[i] = t.getDoubleTopic("SteerClosedLoopError").publish();
 
            }
        }
 
        void publish(SwerveDriveState state, SwerveSubsystem swerve) {
            Pose2d rawPose = state.Pose;
            pose.set(rawPose);

            smoothed.set(swerve.smoothedPose);
            measuredSpeeds.set(state.Speeds);
            fieldSpeeds.set(swerve.getFieldVelocity());
 
            moduleStates.set(state.ModuleStates);
            moduleTargets.set(state.ModuleTargets);
            modulePositions.set(state.ModulePositions);
 
            ChassisSpeeds v = swerve.getFieldVelocity();
            speed.set(Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond));
 
            if (state.OdometryPeriod > 0) {
                odometryHz.set(1.0 / state.OdometryPeriod);
            }
 
            batteryV.set(RobotController.getBatteryVoltage());
            headingDeg.set(rawPose.getRotation().getDegrees());
            rollDeg.set(swerve.getRoll().getDegrees());
            pitchDeg.set(swerve.getPitch().getDegrees());
            yawRateRadS.set(swerve.getRobotVelocity().omegaRadiansPerSecond);
            slipping.set(swerve.currentlySlipping.get());
            slipRecovery.set(swerve.isInSlipRecovery);
            fieldRelative.set(swerve.isFieldRelativeState);
 
            for (int i = 0; i < 4; i++) {
                SwerveModule<?, ?, ?> mod = swerve.drivetrain.getModule(i);
                TalonFX drive = (TalonFX) mod.getDriveMotor();
                TalonFX steer = (TalonFX) mod.getSteerMotor();
 
                driveVelMPS[i].set(state.ModuleStates[i].speedMetersPerSecond);
                driveTargetMPS[i].set(state.ModuleTargets[i].speedMetersPerSecond);
                drivePositionM[i].set(state.ModulePositions[i].distanceMeters);
                driveCurrentA[i].set(drive.getStatorCurrent(false).getValueAsDouble());
                driveVoltageV[i].set(drive.getMotorVoltage(false).getValueAsDouble());
                driveTempC[i].set(drive.getDeviceTemp(false).getValueAsDouble());
                driveCLError[i].set(drive.getClosedLoopError(false).getValueAsDouble());
                driveClosedLoopRef[i].set(drive.getClosedLoopReference(false).getValueAsDouble());
 
                steerAngleDeg[i].set(state.ModuleStates[i].angle.getDegrees());
                steerTargetDeg[i].set(state.ModuleTargets[i].angle.getDegrees());
                steerCurrentA[i].set(steer.getStatorCurrent(false).getValueAsDouble());
                steerVoltageV[i].set(steer.getMotorVoltage(false).getValueAsDouble());
                steerTempC[i].set(steer.getDeviceTemp(false).getValueAsDouble());
                steerCLError[i].set(steer.getClosedLoopError(false).getValueAsDouble());
            }
        }
    }
}
