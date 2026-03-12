package com.spartronics4915.frc2026.subsystems.swerve;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;
import com.spartronics4915.frc2026.Constants.SwerveConstants.SwerveConfigurations;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.simulation.BumpSim;

import static com.spartronics4915.frc2026.Constants.SwerveConstants.*;
import static com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants.driveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class SwerveSubsystem extends SubsystemBase {
    
    public final SwerveDrive swerveDrive;
    public static Pose2d pose;
    public static Pose3d pose3d = new Pose3d();
    public static double movementOverride = 0.0;
    public static boolean isFieldRelative = DEFAULT_IS_FIELD_RELATIVE;
    public static Rotation2d teleopHeadingOffset = Rotation2d.fromDegrees(0.0);
    private final File directory;
    
    private final BumpSim bumpSim;

    public static boolean isRightAlliance;

    private TrapezoidProfile.State yState = new TrapezoidProfile.State();
    private boolean wasOverriding = false;    
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish();
    StructPublisher<Pose3d> pose3dPublisher = NetworkTableInstance.getDefault().getStructTopic("Pose3d", Pose3d.struct).publish();

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

        if (Robot.isSimulation()) {
            bumpSim = new BumpSim(swerveDrive.getSwerveModulePoses(new Pose2d()), 20, 0.1);
        } else {
            bumpSim = null;
        }
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

    public ChassisSpeeds getFieldRelativeVelocity() {
        ChassisSpeeds fieldVelocity = getFieldVelocity();
        if (shouldFlip()) {
            return FlippingUtil.flipFieldSpeeds(fieldVelocity);
        } else {
            return fieldVelocity;
        }
    }

    @Override
    public void periodic() {
        Pose2d pose = getPose();

        posePublisher.accept(pose);

        if (Robot.isSimulation()) {
            Pose3d resolvedPose = bumpSim.resolveRobotPose(pose);
            if (resolvedPose != null) {
                pose3d = resolvedPose;
            }
        } else {
            pose3d = new Pose3d(pose.getX(), pose.getY(), 0, getHeading().rotation);
        }

        pose3dPublisher.accept(pose3d);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public Rotation3d getGyroRotation3d() {
        if (Robot.isSimulation()) {
            return pose3d.getRotation();
        }

        return new Rotation3d(
            swerveDrive.getRoll().getRadians(),
            swerveDrive.getPitch().getRadians(),
            getPose().getRotation().getRadians()
        );
    }

    public RobotHeading getHeading() {
        return new RobotHeading(getGyroRotation3d(), Timer.getFPGATimestamp());
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
        Optional<Pose2d> pose = swerveDrive.swerveDrivePoseEstimator.sampleAt(timestamp);
        if (pose.isEmpty()) {
            System.err.println("Warning: Could not sample pose at timestamp " + timestamp);
            return getPose(); // Return current pose as fallback
        }
        return pose.get();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, visionMeasurementStdDevs);
    }

    public double getSpeed() {
        ChassisSpeeds fieldVelocity = getFieldVelocity();
        return Math.sqrt(fieldVelocity.vxMetersPerSecond * fieldVelocity.vxMetersPerSecond + fieldVelocity.vyMetersPerSecond * fieldVelocity.vyMetersPerSecond);
    }

    public double getMovementOverride() { 
        return movementOverride;
    }

    public void setMovementOverride(double override) {
        movementOverride = override;
    }

    public void lockModules(){
        swerveDrive.lockPose();
    }

    public boolean isFlat() {
        return Math.abs(Rotation2d.fromRadians(getGyroRotation3d().getY()).getDegrees()) < TILT_THRESHOLD_DEGREES
            && Math.abs(Rotation2d.fromRadians(getGyroRotation3d().getX()).getDegrees()) < TILT_THRESHOLD_DEGREES;
    }

    Trigger flatTrigger = new Trigger(this::isFlat).debounce(TILT_DEBOUNCE);

    public boolean isFlatDebounced() {
        return flatTrigger.getAsBoolean();
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

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(Supplier<XboxController> controllerSupplier, BooleanSupplier isFieldRelative, SwerveSubsystem swerve) {
        TrapezoidProfile trapezoidProfile = new TrapezoidProfile(trenchAlignConstraints);
        TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        TrapezoidProfile.State targetState = new TrapezoidProfile.State(0,0);

        return () -> {
            XboxController driverController = controllerSupplier.get();

            Pose2d currentPose = swerve.getPose();
            double override = swerve.getMovementOverride();
            ChassisSpeeds fieldVel = swerve.getFieldVelocity();
            
            // Raw joystick inputs
            double joyVX = applyResponseCurve(MathUtil.applyDeadband(driverController.getLeftY() * -1.0, STICK_DEADBAND)) * MAX_SPEED;
            double joyVY = applyResponseCurve(MathUtil.applyDeadband(driverController.getLeftX() * -1.0, STICK_DEADBAND)) * MAX_SPEED;
            double joyOmega = applyResponseCurve(MathUtil.applyDeadband(driverController.getRightX() * -1.0, STICK_DEADBAND)) * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

            // Determine joystick components in field space
            ChassisSpeeds fieldJoy = ChassisSpeeds.fromRobotRelativeSpeeds(joyVX, joyVY, 0, isFieldRelative.getAsBoolean() ? teleopHeadingOffset : currentPose.getRotation());
            double fieldVX = fieldJoy.vxMetersPerSecond;
            double fieldVY = fieldJoy.vyMetersPerSecond;

            double dt = dtCalc.update();

            if (override != 0.0) {
                if (!swerve.wasOverriding) {
                    swerve.yState.position = currentPose.getY();
                    swerve.yState.velocity = fieldVel.vyMetersPerSecond;
                    swerve.wasOverriding = true;
                }

                double targetX = currentPose.getX();
                double targetTheta = currentPose.getRotation().getRadians();

                targetState.position = override;
                targetState.velocity = 0;

                swerve.yState = trapezoidProfile.calculate(
                    dt, 
                    swerve.yState, 
                    targetState
                );
                double targetY = swerve.yState.position;

                goalState.pose = new Pose2d(targetX, targetY, Rotation2d.fromRadians(targetTheta));
                goalState.fieldSpeeds = new ChassisSpeeds();

                ChassisSpeeds robotTarget = driveController.calculateRobotRelativeSpeeds(currentPose, goalState);
                ChassisSpeeds fieldTarget = ChassisSpeeds.fromRobotRelativeSpeeds(robotTarget, currentPose.getRotation());

                fieldVY = fieldTarget.vyMetersPerSecond;
            } else {
                swerve.wasOverriding = false;
                swerve.yState.position = currentPose.getY();
                swerve.yState.velocity = fieldVel.vyMetersPerSecond;
            }

            // Convert field-relative linear speeds back to robot-relative for the drivetrain
            return ChassisSpeeds.fromFieldRelativeSpeeds(fieldVX, fieldVY, joyOmega, currentPose.getRotation());
        };
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(Supplier<XboxController> controllerSupplier, SwerveSubsystem swerve) {
        return computeVelocitiesFromController(controllerSupplier, () -> isFieldRelative, swerve);
    }

    public static Supplier<ChassisSpeeds> getSwerveTeleopCSSupplier(Supplier<XboxController> controllerSupplier, SwerveSubsystem swerve){
        return computeVelocitiesFromController(controllerSupplier, swerve);
    }

    public Command driveCommand(ChassisSpeeds chassisSpeeds){
        return Commands.runOnce(() -> drive(chassisSpeeds));
    }

    public record RobotHeading(Rotation3d rotation, double timestamp) {}
}
