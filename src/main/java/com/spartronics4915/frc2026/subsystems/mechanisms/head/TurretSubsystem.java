package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import static com.spartronics4915.frc2026.Constants.TurretConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BiConsumer;

public class TurretSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Turret");

    // May need to retune or switch to position + FOC

    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    private CANcoder encoder = new CANcoder(ENCODER_ID, CAN_BUS);
    private final StatusSignal<Angle> motorPositionSignal = motor.getPosition(false);
    private final StatusSignal<Angle> encoderAbsolutePositionSignal =
        encoder.getAbsolutePosition(false);
    private final StatusSignal<Double> dutyCycleSignal = motor.getDutyCycle(false);
    private final BaseStatusSignal[] telemetrySignals = {
        motorPositionSignal,
        encoderAbsolutePositionSignal,
        dutyCycleSignal
    };
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private State targetState = new State();
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private Rotation2d loggedPosition = Rotation2d.kZero;
    private Rotation2d loggedSetpoint = Rotation2d.kZero;
    private Rotation2d profileSetpoint = Rotation2d.kZero;
    private Rotation2d encoderPosition = Rotation2d.kZero;
    private Pose3d mechanismPose = new Pose3d();

    private final PositionTorqueCurrentFOC positionTorqueRequest = new PositionTorqueCurrentFOC(0.0);
    
    private TurretClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    // Vision observer: called during turret's periodic() with accurate timestamp
    private BiConsumer<Rotation2d, Double> visionObserver;
    
    public TurretSubsystem() {

        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
            motorConfigurator.apply(PID_CONFIG);
            motorConfigurator.apply(CURRENT_LIMITS_CONFIG);
            motorConfigurator.apply(FEEDBACK_CONFIG);
            motorConfigurator.apply(MOTOR_OUTPUT_CONFIG); 

        CANcoderConfiguration cancoderConfigurator = new CANcoderConfiguration();
            cancoderConfigurator.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            cancoderConfigurator.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
            cancoderConfigurator.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
            encoder.getConfigurator().apply(cancoderConfigurator);
        
        currentClamp = TurretClamp.UNRESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        encoderAbsolutePositionSignal.refresh();
        Rotation2d initialAngle = Rotation2d.fromRotations(
            encoderAbsolutePositionSignal.getValueAsDouble() * ENCODER_MECHANISM_RATIO
        );
        encoderPosition = initialAngle;
        setMechanismAngle(initialAngle);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> targetState.position, (setpoint) -> setSetpoint(Rotation2d.fromDegrees(setpoint)));

        SmartDashboard.putData("Turret 0", setSetpointCommand(Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Turret 180", setSetpointCommand(Rotation2d.fromDegrees(180)));
    }
 
    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(telemetrySignals);

        targetState.position = MathUtil.clamp(
            targetState.position, 
            minAngle.getRotations(), 
            maxAngle.getRotations()
        );

        if (targetState.position <= minAngle.getRotations() || targetState.position >= maxAngle.getRotations()) {
            targetState.velocity = 0;
        }

        positionTorqueRequest
            .withPosition(targetState.position)
            .withVelocity(targetState.velocity);

        motor.setControl(positionTorqueRequest);

        Rotation2d position = Robot.isSimulation()
            ? Rotation2d.fromRotations(targetState.position)
            : Rotation2d.fromRotations(motorPositionSignal.getValueAsDouble());
        Rotation2d setpoint = Rotation2d.fromRotations(targetState.position);
        appliedDutyCycle = dutyCycleSignal.getValueAsDouble();
        loggedPosition = position;
        loggedSetpoint = setpoint;
        profileSetpoint = setpoint;
        encoderPosition = Rotation2d.fromRotations(
            encoderAbsolutePositionSignal.getValueAsDouble() * ENCODER_MECHANISM_RATIO
        );
        mechanismPose = new Pose3d(
                -0.118295, -0.143695, 0.362276, 
                new Rotation3d(0, 0, position.getRadians()));
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();

        // Notify vision of turret angle with accurate FPGA timestamp
        if (visionObserver != null) {
            visionObserver.accept(position, Timer.getFPGATimestamp());
        }
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("Position", loggedPosition, Rotation2d.struct);
        LOG.critical.log("Setpoint", loggedSetpoint, Rotation2d.struct);
        LOG.critical.log("ProfileSetpoint", profileSetpoint, Rotation2d.struct);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
        LOG.info.log("EncoderPosition", encoderPosition, Rotation2d.struct);
        LOG.debug.log("MechanismPose", mechanismPose);
    }

    public Rotation2d getPosition() {
        if (Robot.isSimulation()) {
            return Rotation2d.fromRotations(targetState.position);
        }
        return loggedPosition;
    }

    public Rotation2d getEncoderPosition() {
        return encoderPosition;
    }

    public Rotation2d getCurrentSetpoint() {
        return Rotation2d.fromRotations(targetState.position);
    }

    public TurretClamp getClamp() {
        return currentClamp;
    }
    
    public void setSetpoint(Rotation2d setpoint){
        targetState.position = setpoint.getRotations();
        targetState.velocity = 0.0;
    }

    public void setComplexSetpoint(Rotation2d setPoint, AngularVelocity velocity){
        targetState.position = setPoint.getRotations();
        targetState.velocity = velocity.in(RotationsPerSecond);
    }

    public void setClamp(TurretClamp clamp){
        currentClamp = clamp;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        loggedPosition = angle;
        resetMechanism(angle);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(Rotation2d angle){
        setSetpoint(angle);
    }

    /**
     * Registers a vision observer to receive turret angle updates with accurate timestamps.
     * Called by VisionSubsystem during initialization.
     * @param observer BiConsumer that takes (turretAngle, fpgaTimestamp)
     */
    public void setVisionObserver(BiConsumer<Rotation2d, Double> observer) {
        this.visionObserver = observer;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setClampCommand(TurretClamp newClamp) {
        return this.runOnce(() -> setClamp(newClamp));
    }

    //#endregion

    public enum TurretClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(230));

        public Rotation2d minAngle;
        public Rotation2d maxAngle;

        private TurretClamp(Rotation2d minAngle, Rotation2d maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}
