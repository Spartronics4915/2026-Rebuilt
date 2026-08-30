package com.spartronics4915.frc2026.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import static com.spartronics4915.frc2026.Constants.PivotConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.LoggedTrapezoidProfile;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PivotSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Pivot");

    // Motion Magic?

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    CANcoder encoder = new CANcoder(ENCODER_ID, CAN_BUS);
    
    LoggedTrapezoidProfile trapProfile = new LoggedTrapezoidProfile(
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private Rotation2d currentSetpoint = new Rotation2d();
    private State currentState = new State();
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private Rotation2d loggedPosition = Rotation2d.kZero;
    private Rotation2d profileSetpoint = Rotation2d.kZero;
    private Rotation2d encoderPosition = Rotation2d.kZero;
    private Pose3d mechanismPose = new Pose3d();

    private final PositionTorqueCurrentFOC positionTorqueRequest = new PositionTorqueCurrentFOC(0.0);

    private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
    private boolean isCharacterizing = false;
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null, 
            null
        ),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
            (log) -> {
                log.motor("Pivot")
                    .voltage(Volts.of(motor.getTorqueCurrent().getValueAsDouble()))
                    .angularPosition(motor.getPosition().getValue())
                    .angularVelocity(motor.getVelocity().getValue())
                    .angularAcceleration(motor.getAcceleration().getValue());
            },
            this
        )
    );

    public PivotSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);
            motorConfig.apply(MOTOR_OUTPUT_CONFIG);

        CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
            cancoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            cancoderConfiguration.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
            cancoderConfiguration.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
            encoder.getConfigurator().apply(cancoderConfiguration);

        StatusSignal<Angle> pos = encoder.getAbsolutePosition();
        pos.waitForUpdate(0.5);
        setMechanismAngle(Rotation2d.fromRotations(pos.getValue().in(Rotations)));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addProfile(trapProfile);
        motor.addSetpoint(() -> currentSetpoint.getDegrees(), (setpoint) -> setSetpoint(Rotation2d.fromDegrees(setpoint)));

        SmartDashboard.putData("Pivot Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Pivot Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("Pivot Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Pivot Dynamic Reverse", sysIdDynamic(Direction.kReverse));

        SmartDashboard.putData("Pivot Ready", setStateCommand(PivotState.READY));
        SmartDashboard.putData("Pivot Safe", setStateCommand(PivotState.SAFE));
        SmartDashboard.putData("Pivot Stow", setStateCommand(PivotState.STOW));
    }

    @Override
    public void periodic(){
        currentState = trapProfile.calculate(
            dtCalc.update(), 
            currentState, 
            new State(currentSetpoint.getRotations(), 0.0)
        );

        positionTorqueRequest.Position = currentState.position;
        if (!isCharacterizing) {
            motor.setControl(positionTorqueRequest);
        }

        loggedPosition = getPosition();
        appliedDutyCycle = motor.getDutyCycle().getValueAsDouble();
        profileSetpoint = Rotation2d.fromRotations(currentState.position);
        encoderPosition = Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
        mechanismPose = new Pose3d(0.2842, 0, 0.1825,
            new Rotation3d(0, -loggedPosition.plus(Rotation2d.fromDegrees(-130)).getRadians(), 0));
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("Position", loggedPosition, Rotation2d.struct);
        LOG.critical.log("Setpoint", currentSetpoint, Rotation2d.struct);
        LOG.critical.log("ProfileSetpoint", profileSetpoint, Rotation2d.struct);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
        LOG.info.log("EncoderPosition", encoderPosition, Rotation2d.struct);
        LOG.debug.log("MechanismPose", mechanismPose);
    }

    public Rotation2d getPosition() {
        if (Robot.isSimulation()) {
            return currentSetpoint;
        }
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public void setSetpoint(Rotation2d setpoint){
        currentSetpoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                setpoint.getRotations(), 
                MIN_ANGLE.getRotations(), 
                MAX_ANGLE.getRotations()
            )
        );
    }

    public Rotation2d getSetpoint() {
        return currentSetpoint;
    }

    public void setState(PivotState state){
        setSetpoint(state.angle);
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        resetMechanism(angle);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(Rotation2d angle) {
        setSetpoint(angle);
        currentState = new State(angle.getRotations(), 0.0);
    }

    // Ignores limits for the purpose of manual reset
    public void deltaSetpoint(Rotation2d delta) {
        currentSetpoint = Rotation2d.fromDegrees(getSetpoint().getDegrees() + delta.getDegrees());
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setStateCommand(PivotState state){
        return setSetpointCommand(state.angle);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction)
            .beforeStarting(() -> isCharacterizing = true)
            .finallyDo(() -> isCharacterizing = false);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction)
            .beforeStarting(() -> isCharacterizing = true)
            .finallyDo(() -> isCharacterizing = false);
    }

    //#endregion
 
    public enum PivotState {
        READY(Rotation2d.fromDegrees(-0.1)),
        SAFE(Rotation2d.fromDegrees(60)),
        STOW(Rotation2d.fromDegrees(130));

        Rotation2d angle;

        private PivotState(Rotation2d angle) {
            this.angle = angle;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}
