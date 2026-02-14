package com.spartronics4915.frc2026.subsystems.mechanisms;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase implements ModeSwitchInterface {

    TalonFX motor = new TalonFX(MOTOR_ID);
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    private double currentSetpoint = 0;
    private State currentState = new State();

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("applied out").publish();
    private final DoublePublisher positionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("position").publish();
    private final DoublePublisher desiredStatePublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("desiredState").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("setpoint").publish();
    
    public ClimberSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motorConfig.apply(motorOutputConfigs);

        setMechanismPosition(0);

        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    //#region Main Functionality

    @Override
    public void periodic(){
        currentSetpoint = MathUtil.clamp(
            currentSetpoint, 
            MIN_HEIGHT, 
            MAX_HEIGHT
        );

        currentState = trapProfile.calculate(
            DELTA_TIME, 
            currentState, 
            new State(currentSetpoint, 0.0)
        );

        PositionVoltage request = new PositionVoltage(currentState.position);
            motor.setControl(request);

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        positionPublisher.accept(getPosition());
        desiredStatePublisher.accept(currentState.position);
        setpointPublisher.accept(currentSetpoint);
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(ClimberState state){
        currentSetpoint = state.position;
    }

    private void setMechanismPosition(double position){
        motor.setPosition(position);
        resetMechanism(position);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(double position){
        currentSetpoint = position;
        currentState = new State(position, 0.0);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setStateCommand(ClimberState state){
        return setSetpointCommand(state.position);
    }

    //#endregion
 
    public enum ClimberState {
        DOWN(0),
        UP(0);

        double position;

        private ClimberState(double position) {
            this.position = position;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}