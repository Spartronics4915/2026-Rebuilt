package com.spartronics4915.frc2026.subsystems.mechanisms;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.PivotConstants.*;
import static edu.wpi.first.units.Units.Rotations;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

public class PivotSubsystem extends SubsystemBase implements ModeSwitchInterface {

    TalonFX motor = new TalonFX(MOTOR_ID, CAN_BUS);
    CANcoder encoder = new CANcoder(ENCODER_ID, CAN_BUS);
    
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private Rotation2d currentSetpoint;
    private State currentState = new State();

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

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motorConfig.apply(motorOutputConfigs);

        CANcoderConfiguration cancoderConfigurator = new CANcoderConfiguration();
        cancoderConfigurator.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            cancoderConfigurator.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
            cancoderConfigurator.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
            encoder.getConfigurator().apply(cancoderConfigurator);

        setMechanismAngle(Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble()));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("Pivot Ready", setStateCommand(PivotState.READY));
        SmartDashboard.putData("Pivot Safe", setStateCommand(PivotState.SAFE));
        SmartDashboard.putData("Pivot Stow", setStateCommand(PivotState.STOW));
    }

    //#region Main Functionality

    @Override
    public void periodic(){
        currentSetpoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                currentSetpoint.getRotations(), 
                MIN_ANGLE.getRotations(), 
                MAX_ANGLE.getRotations()
            )
        );

        currentState = trapProfile.calculate(
            dtCalc.update(), 
            currentState, 
            new State(currentSetpoint.getRotations(), 0.0)
        );

        PositionVoltage request = new PositionVoltage(currentState.position);
        motor.setControl(request);

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
        currentSetpoint = setpoint;
    }

    public void setState(PivotState state){
        currentSetpoint = state.angle;
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        resetMechanism(angle);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(Rotation2d angle){
        currentSetpoint = angle;
        currentState = new State(angle.getRotations(), 0.0);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setStateCommand(PivotState state){
        return setSetpointCommand(state.angle);
    }
 
    public enum PivotState {
        READY(Rotation2d.fromDegrees(4)),
        SAFE(Rotation2d.fromDegrees(10)),
        STOW(Rotation2d.fromDegrees(120));

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
