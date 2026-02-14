package com.spartronics4915.frc2026.subsystems.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.IntakeConstants.*;

import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private TalonFX motor = new TalonFX(MOTOR_ID);

    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new Constraints(
            MAX_VELOCITY, 
            MAX_ACCELERATION
        )
    );

    private State currentState = new State();
    private double currentSetpoint = 0.0;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public IntakeSubsystem() {
        TalonFXConfigurator intakeMotorConfig = motor.getConfigurator();
            intakeMotorConfig.apply(PID_CONFIG);
            intakeMotorConfig.apply(CURRENT_LIMITS_CONFIG);
            intakeMotorConfig.apply(FEEDBACK_CONFIG);

        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("Intake On", setStateCommand(IntakeState.ON));
        SmartDashboard.putData("Intake Off", setStateCommand(IntakeState.OFF));
    }

    @Override
    public void periodic() {
        currentState = trapezoidProfile.calculate(
            DELTA_TIME, 
            currentState, 
            new State(0, currentSetpoint)
        );

        VelocityVoltage request = new VelocityVoltage(currentSetpoint);
            motor.setControl(request);

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPS());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPS() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public void setSetpoint(double newSetpoint){
        currentSetpoint = newSetpoint;
    }

    public void setState(IntakeState newState) {
        setSetpoint(newState.rpm);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(IntakeState state){
        return setSetpointCommand(state.rpm);
    }

    public enum IntakeState {
        ON(100),
        OFF(0);

        double rpm;
        private IntakeState(double rpm) {
            this.rpm = rpm;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(IntakeState.OFF);
    }
    
}