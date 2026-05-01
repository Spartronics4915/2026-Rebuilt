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

import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.LoggedTrapezoidProfile;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PivotSubsystem extends SubsystemBase implements ModeSwitchInterface {

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    CANcoder encoder = new CANcoder(ENCODER_ID, CAN_BUS);
    
    LoggedTrapezoidProfile trapProfile = new LoggedTrapezoidProfile(
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private Rotation2d currentSetpoint = new Rotation2d();
    private State currentState = new State();

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

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("pivot").getDoubleTopic("Applied Out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("pivot").getStructTopic("Position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> desiredStatePublisher = NetworkTableInstance.getDefault().getTable("pivot").getStructTopic("Desired State", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("pivot").getStructTopic("Setpoint", Rotation2d.struct).publish();

    private final StructPublisher<Rotation2d> encoderPositionPublisher = NetworkTableInstance.getDefault().getTable("pivot").getStructTopic("encoder position", Rotation2d.struct).publish();
    
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
        SmartDashboard.putData("Pivot Motor", motor);
    }

    @Override
    public void periodic(){
        // TODO: Is this even needed with PositionTorqueCurrentFOC control?
        currentState = trapProfile.calculate(
            dtCalc.update(), 
            currentState, 
            new State(currentSetpoint.getRotations(), 0.0)
        );

        positionTorqueRequest.Position = currentState.position;
        if (!isCharacterizing) {
            motor.setControl(positionTorqueRequest);
        }

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        positionPublisher.accept(getPosition());
        desiredStatePublisher.accept(Rotation2d.fromRotations(currentState.position));
        setpointPublisher.accept(currentSetpoint);

        encoderPositionPublisher.accept(Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble()));
    }

    public Rotation2d getPosition() {
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
