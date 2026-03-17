package com.spartronics4915.frc2026.subsystems.swerve;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.swerve.SlipDetector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SwerveSubsystem extends SubsystemBase {

    public final SwerveDrivetrain<?, ?, ?> drivetrain;
    private SwerveConfigurations activeConfig;

    private final SlipDetector slipDetector = new SlipDetector();
    private double lastSlipTimestamp = -1.0;
    private boolean isInSlipRecovery = false;

    public static Pose3d pose3d = new Pose3d();
    public static double movementOverride = 0.0;
    public static boolean isFieldRelative = defaultFieldRelative;
    public static Rotation2d teleopHeadingOffset = Rotation2d.fromDegrees(0.0);

    private TrapezoidProfile.State yState = new TrapezoidProfile.State();
    private boolean wasOverriding = false;

    private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.SwerveDriveBrake lockRequest = new SwerveRequest.SwerveDriveBrake();

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> pose3dPublisher = NetworkTableInstance.getDefault().getStructTopic("Pose3d", Pose3d.struct).publish();

    public SwerveSubsystem(SwerveConfigurations config) {
        drivetrain = new SwerveDrivetrain<>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            config.drivetrainConstants,
            config.modules[0], config.modules[1], config.modules[2], config.modules[3]
        );
        drivetrain.setStateStdDevs(normalStdDevs);
        activeConfig = config;

        if (Utils.isSimulation()) {
            drivetrain.resetPose(
                new Pose2d(new Translation2d(2.0, 5.0), Rotation2d.fromDegrees(120))
            );
        }

        drivetrain.registerTelemetry(this::updateOdometry);

        configurePathPlanner();
    }

    private void updateOdometry(SwerveDriveState state) {
        boolean slipping = slipDetector.update(state.ModuleStates, state.ModuleTargets);
        double present = Utils.getCurrentTimeSeconds();

        if (slipping) {
            lastSlipTimestamp = present;
            if (!isInSlipRecovery) {
                isInSlipRecovery = true;
                drivetrain.setStateStdDevs(slipStdDevs);
            }
        } else if (isInSlipRecovery && (present - lastSlipTimestamp) > slipRecoverySeconds) {
            isInSlipRecovery = false;
            drivetrain.setStateStdDevs(normalStdDevs);
        }
    }

    /** Must be called from Robot.simulationPeriodic(). */
    public void simulationPeriodic() {
        drivetrain.updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();
        posePublisher.set(pose);
        pose3d = new Pose3d(
            pose.getX(), pose.getY(), 0,
            new Rotation3d(getPitch().getRadians(), getRoll().getRadians(),
                pose.getRotation().getRadians())
        );
        pose3dPublisher.set(pose3d);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drivetrain.setControl(driveRequest.withSpeeds(chassisSpeeds));
    }

    public void lockModules() {
        drivetrain.setControl(lockRequest);
    }

    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

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
        drivetrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
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

    public boolean isSlipping() {
        return isInSlipRecovery;
    }

    public boolean shouldFlip() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public double getMovementOverride() {
        return movementOverride;
    }

    public void setMovementOverride(double override) {
        movementOverride = override;
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(
        Supplier<XboxController> controllerSupplier,
        BooleanSupplier isFieldRelativeSupplier,
        SwerveSubsystem swerve
    ) {
        TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trenchAlignConstraints);
        TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        TrapezoidProfile.State targetState = new TrapezoidProfile.State();

        return () -> {
            XboxController controller = controllerSupplier.get();
            Pose2d currentPose = swerve.getPose();
            double override = swerve.getMovementOverride();
            ChassisSpeeds fieldVel = swerve.getFieldVelocity();

            double joyVX = applyResponseCurve(
                MathUtil.applyDeadband(controller.getLeftY() * -1.0, stickDeadband)) * maxSpeed;
            double joyVY = applyResponseCurve(
                MathUtil.applyDeadband(controller.getLeftX() * -1.0, stickDeadband)) * maxSpeed;
            double joyOmega = applyResponseCurve(
                MathUtil.applyDeadband(controller.getRightX() * -1.0, stickDeadband)) * maxAngularSpeed.in(RadiansPerSecond);

            ChassisSpeeds fieldJoy = ChassisSpeeds.fromRobotRelativeSpeeds(
                joyVX, joyVY, 0,
                isFieldRelativeSupplier.getAsBoolean()
                    ? teleopHeadingOffset
                    : currentPose.getRotation()
            );
            double fieldVX = fieldJoy.vxMetersPerSecond;
            double fieldVY = fieldJoy.vyMetersPerSecond;

            //if (driverController.getRightTriggerAxis() > 0.5 && Math.hypot(fieldVX, fieldVY) > 0.1) {
            //    if (!swerve.wasAligning) {
            //        headingController.reset();
            //        swerve.wasAligning = true;
            //    }
            //    Rotation2d motionAngle = new Rotation2d(fieldVX, fieldVY);
            //    double currentAngle = currentPose.getRotation().getRadians();
            //    joyOmega = headingController.calculate(currentAngle, motionAngle.getRadians());
            //} else {
            //    swerve.wasAligning = false;
            //}

            double dt = dtCalc.update();

            if (override != 0.0) {
                if (!swerve.wasOverriding) {
                    swerve.yState.position = currentPose.getY();
                    swerve.yState.velocity = fieldVel.vyMetersPerSecond;
                    swerve.wasOverriding = true;
                }

                targetState.position = override;
                targetState.velocity = 0;

                swerve.yState = trapezoidProfile.calculate(dt, swerve.yState, targetState);

                goalState.pose = new Pose2d(
                    currentPose.getX(),
                    swerve.yState.position,
                    currentPose.getRotation()
                );
                goalState.fieldSpeeds = new ChassisSpeeds();

                ChassisSpeeds robotTarget = driveController.calculateRobotRelativeSpeeds(
                    currentPose, goalState);
                fieldVY = ChassisSpeeds.fromRobotRelativeSpeeds(
                    robotTarget, currentPose.getRotation()).vyMetersPerSecond;
            } else {
                swerve.wasOverriding = false;
                swerve.yState.position = currentPose.getY();
                swerve.yState.velocity = fieldVel.vyMetersPerSecond;
            }

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldVX, fieldVY, joyOmega, currentPose.getRotation());
        };
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(
        Supplier<XboxController> controllerSupplier, SwerveSubsystem swerve
    ) {
        return computeVelocitiesFromController(
            controllerSupplier, () -> isFieldRelative, swerve);
    }

    public static Supplier<ChassisSpeeds> getSwerveTeleopCSSupplier(
        Supplier<XboxController> controllerSupplier, SwerveSubsystem swerve
    ) {
        return computeVelocitiesFromController(controllerSupplier, swerve);
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

    public Command driveCommand(ChassisSpeeds chassisSpeeds) {
        return Commands.runOnce(() -> drive(chassisSpeeds));
    }

    private static double applyResponseCurve(double x) {
        return Math.signum(x) * (x * x);
    }

    public static ChassisSpeeds rotateLinearChassisSpeeds(ChassisSpeeds in, Rotation2d offset) {
        Translation2d linear = new Translation2d(
            in.vxMetersPerSecond, in.vyMetersPerSecond).rotateBy(offset);
        return new ChassisSpeeds(linear.getX(), linear.getY(), in.omegaRadiansPerSecond);
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}

}