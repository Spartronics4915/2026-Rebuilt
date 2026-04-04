package com.spartronics4915.frc2026.subsystems.mechanisms;

import static com.spartronics4915.frc2026.Constants.ClimberConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.LoggedTrapezoidProfile;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements ModeSwitchInterface {

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    LoggedTrapezoidProfile trapProfile = new LoggedTrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private double currentSetpoint;
    private State currentState = new State();

    private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("applied out").publish();
    private final DoublePublisher positionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("position").publish();
    private final DoublePublisher desiredStatePublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("desiredState").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("setpoint").publish();
    
    public ClimberSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);
            motorConfig.apply(MOTOR_OUTPUT_CONFIG);

        setMechanismPosition(getPosition());
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addProfile(trapProfile);
        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Climber Climb", setStateCommand(ClimberState.JORBIT));
        SmartDashboard.putData("Climber Down", setStateCommand(ClimberState.DOWN));
        SmartDashboard.putData("Climber Motor", motor);
    }

    @Override
    public void periodic(){
        currentSetpoint = MathUtil.clamp(
            currentSetpoint, 
            MIN_HEIGHT, 
            MAX_HEIGHT
        );

        currentState = trapProfile.calculate(
            dtCalc.update(), 
            currentState, 
            new State(currentSetpoint, 0.0)
        );

        positionVoltage.withEnableFOC(ENABLE_FOC).Position = currentState.position;
        motor.setControl(positionVoltage);

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        positionPublisher.accept(getPosition());
        desiredStatePublisher.accept(currentState.position);
        setpointPublisher.accept(currentSetpoint);
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
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
        JORBIT(2.7);

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
