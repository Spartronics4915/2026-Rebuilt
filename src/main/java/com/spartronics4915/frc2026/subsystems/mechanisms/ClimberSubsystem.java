package com.spartronics4915.frc2026.subsystems.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.MotorHelpers.CTRE.LoggedTalonFX;
import com.spartronics4915.frc2026.util.MotorHelpers.LoggedTrapezoidProfile;
import com.spartronics4915.frc2026.util.TimeVarianceAuthority;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ClimberConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

public class ClimberSubsystem extends SubsystemBase implements ModeSwitchInterface {

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    LoggedTrapezoidProfile trapProfile = new LoggedTrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private double currentSetpoint;
    private State currentState = new State();

    private static final PositionVoltage positionVoltage = new PositionVoltage(0.0);

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("applied out").publish();
    private final DoublePublisher positionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("position").publish();
    private final DoublePublisher desiredStatePublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("desiredState").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("setpoint").publish();

    private final StructPublisher<Pose3d> componentPosePublisher = NetworkTableInstance.getDefault().getTable("climber").getStructTopic("Climber Component", Pose3d.struct).publish();
    
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

    //#region Main Functionality

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

        positionVoltage.Position = currentState.position;
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
        JORBIT(2.8);

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
