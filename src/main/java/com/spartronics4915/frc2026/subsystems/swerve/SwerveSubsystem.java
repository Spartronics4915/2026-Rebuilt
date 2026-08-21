package com.spartronics4915.frc2026.subsystems.swerve;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.ArrayDeque;
import java.util.Objects;
import java.util.OptionalDouble;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.autos.Autos;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/**
 * Swerve drivetrain subsystem.
 *
 * <p>CTRE owns the hardware-level drivetrain and pose estimator. This class owns
 * robot-level drivetrain policy: teleop input shaping, operator perspective,
 * CTRE heading control, autonomous requests, vision integration, and telemetry.</p>
 */
public class SwerveSubsystem extends SubsystemBase {
    private static final TimeVarianceAuthority TVA = new TimeVarianceAuthority();

    private static SwerveSubsystem instance;

    private final SwerveConfigurations configuration;
    private final SwerveDrivetrain<?, ?, ?> drivetrain;
    private Alliance appliedAlliance = null;

    private final SwerveRequest.FieldCentric fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.FieldCentricFacingAngle headingLockRequest =
        new SwerveRequest.FieldCentricFacingAngle().withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective)
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

    private final SwerveRequest.SwerveDriveBrake lockRequest = new SwerveRequest.SwerveDriveBrake();

    private final ConcurrentYawRateBuffer yawRateBuffer = new ConcurrentYawRateBuffer(1.0);
    private double prevYawRad = Double.NaN;
    private double prevYawTimestamp = Double.NaN;

    private final Field2d field = new Field2d();

    private final Debouncer flatDebouncer = new Debouncer(TILT_DEBOUNCE);
    private boolean isFlatDebouncedValue = false;

    private volatile double lastDriveCommandTimestamp = Utils.getCurrentTimeSeconds();

    private boolean isFieldRelativeState = DEFAULT_FIELD_RELATIVE;
    private Rotation2d teleopHeadingOffset = Rotation2d.kZero;

    /**
     * Mechanism/3D visualization pose. This is derived from the authoritative
     * drivetrain estimator pose; it is not an independent simulation truth pose.
     */
    private Pose3d pose3d = new Pose3d();

    private double movementOverride = 0.0;

    /* Teleop state */
    private Rotation2d lockedHeading = null;
    @SuppressWarnings("unused")
    private boolean wasOverriding = false;

    /* Trench / movement override controller */
    private final TrapezoidProfileStateController overrideController = new TrapezoidProfileStateController();

    private final SwerveTelemetry telemetry = new SwerveTelemetry();

    public SwerveSubsystem(SwerveConfigurations config) {
        configuration = Objects.requireNonNull(config, "config");

        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            config.drivetrainConstants,
            ODOMETRY_FREQUENCY,
            config.modules[0],
            config.modules[1],
            config.modules[2],
            config.modules[3]
        );

        drivetrain.setStateStdDevs(NORMAL_STD_DEVS);
        drivetrain.configNeutralMode(NeutralModeValue.Brake);

        headingLockRequest.HeadingController.setPID(HEADING_LOCK_P, 0.0, HEADING_LOCK_D);
        headingLockRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        if (Robot.isSimulation()) {
            drivetrain.resetPose(
                new Pose2d(
                    new Translation2d(14.0, 5.0),
                    Rotation2d.fromDegrees(180.0)
                )
            );
        }

        drivetrain.registerTelemetry(this::updateOdometry);
        configureBLine();

        SmartDashboard.putData(field);
    }

    public static synchronized SwerveSubsystem getInstance(SwerveConfigurations config) {
        if (instance == null) {
            instance = new SwerveSubsystem(config);
        } else if (instance.configuration != config && !instance.configuration.equals(config)) {
            throw new IllegalStateException(
                "SwerveSubsystem was already initialized with a different configuration."
            );
        }

        return instance;
    }

    private void updateOperatorPerspective() {
        DriverStation.getAlliance().ifPresent(alliance -> {
            if (DriverStation.isDisabled() || alliance != appliedAlliance) {
                drivetrain.setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Red
                        ? RED_OPERATOR_FORWARD
                        : BLUE_OPERATOR_FORWARD
                );
                appliedAlliance = alliance;
            }
        });
    }

    private void updateOdometry(SwerveDriveState state) {
        double nowYaw = state.Pose.getRotation().getRadians();
        double nowTimestamp = state.Timestamp;

        if (!Double.isNaN(prevYawTimestamp)) {
            double dt = nowTimestamp - prevYawTimestamp;
            if (dt > 0.0) {
                double yawRate = MathUtil.angleModulus(nowYaw - prevYawRad) / dt;
                yawRateBuffer.addSample(nowTimestamp, yawRate);
            }
        }

        prevYawRad = nowYaw;
        prevYawTimestamp = nowTimestamp;
    }

    @Override
    public void periodic() {
        updateOperatorPerspective();
        double now = Utils.getCurrentTimeSeconds();

        isFlatDebouncedValue = flatDebouncer.calculate(isFlat());

        /*
         * The CTRE drivetrain estimator is the sole planar pose source.
         * VisionSystemSim also consumes getPose() directly.
         */
        pose3d = new Pose3d(getPose());

        if ((now - lastDriveCommandTimestamp) > STALE_COMMAND_TIMEOUT) {
            stop();
        }

        telemetry.publish(drivetrain.getState(), this);

        field.setRobotPose(getPose());

        if (DriverStation.isDisabled()) {
            Autos.setSwervePose(getPose());
        }
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(
            TVA.update(), // Gets the actual delta time, not predicted
            RobotController.getBatteryVoltage()
        );

        /* No independent simulated truth pose is maintained. */
        pose3d = new Pose3d(getPose());
    }

    /**
     * Converts raw normalized Xbox inputs into the robot's teleop drive behavior.
     *
     * <p>This is intentionally the single entry point for human-driven swerve
     * control. The command layer should only provide controller inputs.</p>
     */
    public void acceptTeleopInput(double rawVX, double rawVY, double rawOmega) {
        double vX = shapeJoystick(rawVX) * MAX_VELOCITY;
        double vY = shapeJoystick(rawVY) * MAX_VELOCITY;
        double omega = shapeJoystick(rawOmega) * MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

        if (movementOverride != 0.0) {
            vY = computeOverrideVY(movementOverride);
            wasOverriding = true;
        } else {
            wasOverriding = false;
        }

        if (!isFieldRelative()) {
            lockedHeading = null;
            driveRobotCentric(vX, vY, omega);
            return;
        }

        double rotationThreshold = MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * 0.03;
        boolean driverIsRotating = Math.abs(omega) > rotationThreshold;

        double translationMagnitude = Math.hypot(vX, vY);
        boolean driverIsTranslating = translationMagnitude > MAX_VELOCITY * 0.05;

        if (driverIsRotating) {
            lockedHeading = null;
            driveFieldCentric(vX, vY, omega);
            return;
        }

        if (driverIsTranslating) {
            if (lockedHeading == null) {
                lockedHeading = getPose().getRotation().minus(getHeadingOffset());
            }

            driveFieldCentricFacingAngle(vX, vY, lockedHeading);
            return;
        }

        lockedHeading = null;
        stop();
    }

    private static double shapeJoystick(double value) {
        double deadbanded = MathUtil.applyDeadband(value, STICK_DEADBAND);
        return Math.signum(deadbanded) * Math.pow(Math.abs(deadbanded), 1.5);
    }

    private double computeOverrideVY(double overrideY) {
        return overrideController.calculate(this, overrideY);
    }

    public void driveFieldCentric(double vX, double vY, double omega) {
        lastDriveCommandTimestamp = Utils.getCurrentTimeSeconds();

        drivetrain.setControl(
            fieldCentricRequest
                .withVelocityX(vX)
                .withVelocityY(vY)
                .withRotationalRate(omega)
        );
    }

    public void driveFieldCentricFacingAngle(
        double vX,
        double vY,
        Rotation2d targetHeading
    ) {
        lastDriveCommandTimestamp = Utils.getCurrentTimeSeconds();
        drivetrain.setControl(
            headingLockRequest
                .withVelocityX(vX)
                .withVelocityY(vY)
                .withTargetDirection(targetHeading)
        );
    }

    public void driveRobotCentric(double vX, double vY, double omega) {
        lastDriveCommandTimestamp = Utils.getCurrentTimeSeconds();
        drivetrain.setControl(
            robotCentricRequest
                .withVelocityX(vX)
                .withVelocityY(vY)
                .withRotationalRate(omega)
        );
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        driveRobotCentric(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond
        );
    }

    public void stop() {
        lastDriveCommandTimestamp = Utils.getCurrentTimeSeconds();
        drivetrain.setControl(
            autoRequest.withSpeeds(new ChassisSpeeds())
        );
    }

    public void lockModules() {
        lastDriveCommandTimestamp = Utils.getCurrentTimeSeconds();
        drivetrain.setControl(lockRequest);
    }

    public void setDriverPerspective(Rotation2d heading) {
        drivetrain.setOperatorPerspectiveForward(heading);
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    /** Pose flipped to the current alliance's perspective. Use for game logic. */
    public Pose2d getRelativePose() {
        Pose2d pose = getPose();
        return Autos.shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public Pose2d getPastVisionPose(double timestamp) {
        return drivetrain.samplePoseAt(timestamp).orElseGet(this::getPose);
    }

    public void addVisionMeasurement(
        Pose2d pose,
        double timestamp,
        Matrix<N3, N1> stdDevs
    ) {
        drivetrain.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void resetPose(Pose2d pose) {
        if (pose == null) {
            return;
        }

        drivetrain.resetPose(pose);
    }

    public ChassisSpeeds getRobotVelocity() {
        return drivetrain.getState().Speeds;
    }

    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getRobotVelocity(),
            getPose().getRotation()
        );
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        ChassisSpeeds velocity = getFieldVelocity();
        return Autos.shouldFlip()
            ? FlippingUtil.flipFieldSpeeds(velocity)
            : velocity;
    }

    public double getSpeed() {
        ChassisSpeeds velocity = getFieldVelocity();
        return Math.hypot(
            velocity.vxMetersPerSecond,
            velocity.vyMetersPerSecond
        );
    }

    public Rotation2d getRoll() {
        if (Robot.isSimulation()) {
            return new Rotation2d(pose3d.getRotation().getX());
        }

        return Rotation2d.fromDegrees(
            drivetrain.getPigeon2().getRoll().getValueAsDouble()
        );
    }

    public Rotation2d getPitch() {
        if (Robot.isSimulation()) {
            return new Rotation2d(pose3d.getRotation().getY());
        }

        return Rotation2d.fromDegrees(
            drivetrain.getPigeon2().getPitch().getValueAsDouble()
        );
    }

    public Rotation3d getGyroRotation3d() {
        if (Robot.isSimulation()) {
            return pose3d.getRotation();
        }

        return drivetrain.getRotation3d();
    }

    public RobotHeading getHeading() {
        return new RobotHeading(
            getGyroRotation3d(),
            Timer.getFPGATimestamp()
        );
    }

    public OptionalDouble getMaxAbsYawRateInRange(
        double minTime,
        double maxTime
    ) {
        return yawRateBuffer.getMaxAbsValueInRange(minTime, maxTime);
    }

    public boolean isFlat() {
        return Math.abs(getPitch().getDegrees()) < TILT_THRESHOLD_DEGREES
            && Math.abs(getRoll().getDegrees()) < TILT_THRESHOLD_DEGREES;
    }

    public boolean isFlatDebounced() {
        return isFlatDebouncedValue;
    }

    public boolean isFieldRelative() {
        return isFieldRelativeState;
    }

    public void setFieldRelative(boolean fieldRelative) {
        isFieldRelativeState = fieldRelative;
        lockedHeading = null;
    }

    public void toggleFieldRelative() {
        setFieldRelative(!isFieldRelativeState);
    }

    public Rotation2d getHeadingOffset() {
        return teleopHeadingOffset.plus(
            Autos.shouldFlip() ? Rotation2d.kPi : Rotation2d.kZero
        );
    }

    public void resetHeadingOffset() {
        teleopHeadingOffset = getRelativePose().getRotation();
        lockedHeading = null;
        setDriverPerspective(getHeadingOffset());
    }

    public double getMovementOverride() {
        return movementOverride;
    }

    public void setMovementOverride(double override) {
        if (MathUtil.applyDeadband(override, 1e-9) == 0.0) {
            movementOverride = 0.0;
            return;
        }

        movementOverride = override;
    }

    private void configureBLine() {
        var targetPosePub = NetworkTableInstance.getDefault()
            .getStructTopic(
                "SmartDashboard/BLine/Target Pose",
                Pose2d.struct
            )
            .publish();

        var targetPathPub = NetworkTableInstance.getDefault()
            .getStructArrayTopic(
                "SmartDashboard/BLine/Target Path",
                Pose2d.struct
            )
            .publish();

        FollowPath.setDoubleLoggingConsumer(pair ->
            SmartDashboard.putNumber(
                "BLine/" + pair.getFirst(),
                pair.getSecond()
            )
        );

        FollowPath.setBooleanLoggingConsumer(pair ->
            SmartDashboard.putBoolean(
                "BLine/" + pair.getFirst(),
                pair.getSecond()
            )
        );

        FollowPath.setPoseLoggingConsumer(pair ->
            targetPosePub.set(pair.getSecond())
        );

        FollowPath.setTranslationListLoggingConsumer(pair -> {
            Translation2d[] translations = pair.getSecond();
            Pose2d[] poses = new Pose2d[translations.length];

            for (int i = 0; i < translations.length; i++) {
                poses[i] = new Pose2d(
                    translations[i],
                    Rotation2d.kZero
                );
            }

            targetPathPub.set(poses);
        });

        FollowPath.Builder pathBuilder = new FollowPath.Builder(
            this,
            this::getPose,
            this::getRobotVelocity,
            this::drive,
            translationPID,
            rotationPID,
            crossTrackPID
        ).withDefaultShouldFlip();

        Autos.setPathBuilder(pathBuilder);
        Path.setDefaultGlobalConstraints(defaultPathConstraints);
    }

    public static ChassisSpeeds rotateLinearChassisSpeeds(
        ChassisSpeeds in,
        Rotation2d offset
    ) {
        Translation2d linear = new Translation2d(
            in.vxMetersPerSecond,
            in.vyMetersPerSecond
        ).rotateBy(offset);

        return new ChassisSpeeds(
            linear.getX(),
            linear.getY(),
            in.omegaRadiansPerSecond
        );
    }

    public void configureStdDevsDisabled() {
        drivetrain.setStateStdDevs(
            VecBuilder.fill(1.0, 1.0, 1.0)
        );
    }

    public void configureStdDevsEnabled() {
        drivetrain.setStateStdDevs(NORMAL_STD_DEVS);
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}

    /**
     * Small local controller for the existing trench movement override.
     */
    private static final class TrapezoidProfileStateController {
        private final TimeVarianceAuthority dtAuthority = new TimeVarianceAuthority();
        private final TrapezoidProfile profile = new TrapezoidProfile(TRENCH_ALIGN_CONSTRAINTS);
        private final PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        private final TrapezoidProfile.State targetState = new TrapezoidProfile.State();
        private TrapezoidProfile.State yState = new TrapezoidProfile.State();
        private boolean initialized;

        private final PPHolonomicDriveController controller =
            new PPHolonomicDriveController(
                alignTranslationPID,
                alignRotationPID
            );

        double calculate(SwerveSubsystem swerve, double targetY) {
            double dt = dtAuthority.update();
            ChassisSpeeds fieldVelocity = swerve.getFieldVelocity();

            if (!initialized) {
                yState.position = swerve.getRelativePose().getY();
                yState.velocity = fieldVelocity.vyMetersPerSecond;
                initialized = true;
            }

            targetState.position = targetY;
            targetState.velocity = 0.0;

            yState = profile.calculate(
                dt,
                yState,
                targetState
            );

            Pose2d currentPose = swerve.getRelativePose();

            goalState.pose = new Pose2d(
                currentPose.getX(),
                yState.position,
                currentPose.getRotation()
            );
            goalState.fieldSpeeds = new ChassisSpeeds();

            ChassisSpeeds robotTarget =
                controller.calculateRobotRelativeSpeeds(
                    currentPose,
                    goalState
                );

            double cosTheta = currentPose.getRotation().getCos();
            double sinTheta = currentPose.getRotation().getSin();

            return robotTarget.vxMetersPerSecond * sinTheta
                + robotTarget.vyMetersPerSecond * cosTheta;
        }

        @SuppressWarnings("unused")
        void reset() {
            initialized = false;
            yState = new edu.wpi.first.math.trajectory.TrapezoidProfile.State();
        }
    }

    /**
     * Minimal synchronized time-series buffer for yaw-rate history.
     * Kept local so the drivetrain API doesn't depend on the old slip/vision utility
     * package solely for this measurement
     */
    private static final class ConcurrentYawRateBuffer {
        private final double retentionSeconds;
        private final ArrayDeque<double[]> samples = new ArrayDeque<>();

        ConcurrentYawRateBuffer(double retentionSeconds) {
            this.retentionSeconds = retentionSeconds;
        }

        synchronized void addSample(double timestamp, double value) {
            samples.addLast(new double[] {timestamp, value});
            double cutoff = timestamp - retentionSeconds;
            while (!samples.isEmpty() && samples.peekFirst()[0] < cutoff) {
                samples.removeFirst();
            }
        }

        synchronized OptionalDouble getMaxAbsValueInRange(
            double minTime,
            double maxTime
        ) {
            double max = 0.0;
            boolean found = false;

            for (double[] sample : samples) {
                if (sample[0] >= minTime && sample[0] <= maxTime) {
                    max = Math.max(max, Math.abs(sample[1]));
                    found = true;
                }
            }

            return found ? OptionalDouble.of(max) : OptionalDouble.empty();
        }
    }

    private final class SwerveTelemetry {
        private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

        private final NetworkTable root = NetworkTableInstance.getDefault().getTable("swerve");

        private final StructPublisher<Pose2d> pose = root.getStructTopic("Pose", Pose2d.struct).publish();
        private final StructPublisher<Pose3d> pose3dPublisher = root.getStructTopic("Pose3d", Pose3d.struct).publish();
        private final StructPublisher<ChassisSpeeds> measuredSpeeds = root.getStructTopic("MeasuredSpeeds", ChassisSpeeds.struct).publish();
        private final StructPublisher<ChassisSpeeds> fieldSpeeds = root.getStructTopic("FieldRelativeSpeeds", ChassisSpeeds.struct).publish();

        private final StructArrayPublisher<SwerveModuleState> moduleStates =
            root.getStructArrayTopic(
                "ModuleStates",
                SwerveModuleState.struct
            ).publish();
        private final StructArrayPublisher<SwerveModuleState> moduleTargets =
            root.getStructArrayTopic(
                "ModuleTargets",
                SwerveModuleState.struct
            ).publish();
        private final StructArrayPublisher<SwerveModulePosition> modulePositions =
            root.getStructArrayTopic(
                "ModulePositions",
                SwerveModulePosition.struct
            ).publish();

        private final DoublePublisher speed = root.getDoubleTopic("SpeedMPS").publish();
        private final DoublePublisher odometryHz = root.getDoubleTopic("OdometryHz").publish();
        private final DoublePublisher batteryVoltage = root.getDoubleTopic("BatteryVoltage").publish();
        private final DoublePublisher headingDeg = root.getDoubleTopic("HeadingDeg").publish();
        private final DoublePublisher rollDeg = root.getDoubleTopic("RollDeg").publish();
        private final DoublePublisher pitchDeg = root.getDoubleTopic("PitchDeg").publish();
        private final DoublePublisher yawRateRadS = root.getDoubleTopic("YawRateRadS").publish();
        private final BooleanPublisher fieldRelative = root.getBooleanTopic("IsFieldRelative").publish();
        private final BooleanPublisher flat = root.getBooleanTopic("IsFlat").publish();

        private final DoublePublisher[] driveVelocity = new DoublePublisher[4];
        private final DoublePublisher[] driveTarget = new DoublePublisher[4];
        private final DoublePublisher[] drivePosition = new DoublePublisher[4];
        private final DoublePublisher[] driveCurrent = new DoublePublisher[4];
        private final DoublePublisher[] driveVoltage = new DoublePublisher[4];
        private final DoublePublisher[] driveTemperature = new DoublePublisher[4];
        private final DoublePublisher[] driveClosedLoopError = new DoublePublisher[4];
        private final DoublePublisher[] driveClosedLoopReference = new DoublePublisher[4];

        private final DoublePublisher[] steerAngle = new DoublePublisher[4];
        private final DoublePublisher[] steerTarget = new DoublePublisher[4];
        private final DoublePublisher[] steerCurrent = new DoublePublisher[4];
        private final DoublePublisher[] steerVoltage = new DoublePublisher[4];
        private final DoublePublisher[] steerTemperature = new DoublePublisher[4];
        private final DoublePublisher[] steerClosedLoopError = new DoublePublisher[4];

        SwerveTelemetry() {
            for (int i = 0; i < 4; i++) {
                NetworkTable table =
                    root.getSubTable("modules/" + MODULE_NAMES[i]);

                driveVelocity[i] = table.getDoubleTopic("DriveVelocityMPS").publish();
                driveTarget[i] = table.getDoubleTopic("DriveTargetMPS").publish();
                drivePosition[i] = table.getDoubleTopic("DrivePositionM").publish();
                driveCurrent[i] = table.getDoubleTopic("DriveCurrentA").publish();
                driveVoltage[i] = table.getDoubleTopic("DriveVoltageV").publish();
                driveTemperature[i] = table.getDoubleTopic("DriveTempC").publish();
                driveClosedLoopError[i] = table.getDoubleTopic("DriveClosedLoopError").publish();
                driveClosedLoopReference[i] = table.getDoubleTopic("DriveClosedLoopRef").publish();

                steerAngle[i] = table.getDoubleTopic("SteerAngleDeg").publish();
                steerTarget[i] = table.getDoubleTopic("SteerTargetDeg").publish();
                steerCurrent[i] = table.getDoubleTopic("SteerCurrentA").publish();
                steerVoltage[i] = table.getDoubleTopic("SteerVoltageV").publish();
                steerTemperature[i] = table.getDoubleTopic("SteerTempC").publish();
                steerClosedLoopError[i] = table.getDoubleTopic("SteerClosedLoopError").publish();
            }
        }

        void publish(
            SwerveDriveState state,
            SwerveSubsystem swerve
        ) {
            Pose2d rawPose = state.Pose;

            pose.set(rawPose);
            pose3dPublisher.set(swerve.pose3d);
            measuredSpeeds.set(state.Speeds);
            fieldSpeeds.set(swerve.getFieldVelocity());

            moduleStates.set(state.ModuleStates);
            moduleTargets.set(state.ModuleTargets);
            modulePositions.set(state.ModulePositions);

            ChassisSpeeds fieldVelocity = swerve.getFieldVelocity();
            speed.set(
                Math.hypot(
                    fieldVelocity.vxMetersPerSecond,
                    fieldVelocity.vyMetersPerSecond
                )
            );

            if (state.OdometryPeriod > 0.0) {
                odometryHz.set(1.0 / state.OdometryPeriod);
            }

            batteryVoltage.set(RobotController.getBatteryVoltage());
            headingDeg.set(rawPose.getRotation().getDegrees());
            rollDeg.set(swerve.getRoll().getDegrees());
            pitchDeg.set(swerve.getPitch().getDegrees());
            yawRateRadS.set(swerve.getRobotVelocity().omegaRadiansPerSecond);
            fieldRelative.set(swerve.isFieldRelativeState);
            flat.set(swerve.isFlatDebounced());

            for (int i = 0; i < 4; i++) {
                SwerveModule<?, ?, ?> module =
                    swerve.drivetrain.getModule(i);

                TalonFX drive = (TalonFX) module.getDriveMotor();
                TalonFX steer = (TalonFX) module.getSteerMotor();

                driveVelocity[i].set(state.ModuleStates[i].speedMetersPerSecond);
                driveTarget[i].set(state.ModuleTargets[i].speedMetersPerSecond);
                drivePosition[i].set(state.ModulePositions[i].distanceMeters);
                driveCurrent[i].set(drive.getStatorCurrent().getValueAsDouble());
                driveVoltage[i].set(drive.getMotorVoltage().getValueAsDouble());
                driveTemperature[i].set(drive.getDeviceTemp().getValueAsDouble());
                driveClosedLoopError[i].set(drive.getClosedLoopError().getValueAsDouble());
                driveClosedLoopReference[i].set(drive.getClosedLoopReference().getValueAsDouble());

                steerAngle[i].set(state.ModuleStates[i].angle.getDegrees());
                steerTarget[i].set(state.ModuleTargets[i].angle.getDegrees());
                steerCurrent[i].set(steer.getStatorCurrent().getValueAsDouble());
                steerVoltage[i].set(steer.getMotorVoltage().getValueAsDouble());
                steerTemperature[i].set(steer.getDeviceTemp().getValueAsDouble());
                steerClosedLoopError[i].set(steer.getClosedLoopError().getValueAsDouble());
            }
        }
    }
}