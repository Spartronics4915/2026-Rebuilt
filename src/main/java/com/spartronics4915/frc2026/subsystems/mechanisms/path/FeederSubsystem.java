package com.spartronics4915.frc2026.subsystems.mechanisms.path;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.FeederConstants.*;

public class FeederSubsystem extends SubsystemBase {
    
    private TalonFX motor;

    private double currentSetpoint;

    private DoublePublisher rpmPublisher = NetworkTableInstance.getDefault().getDoubleTopic("RPM").publish();
    private DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Setpoint").publish();

    //#region Main Functionality

    public FeederSubsystem() {
        this.currentSetpoint = 0.0;

        motor = new TalonFX(MOTOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
        
    }

    @Override
    public void periodic() {
        VelocityVoltage request = new VelocityVoltage(currentSetpoint);
        motor.setControl(request);

        rpmPublisher.accept(getCurrentRPM());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(FeederState state) {
        setSetpoint(state.rpm);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(FeederState state){
        return setSetpointCommand(state.rpm);
    }

    public enum FeederState{
        ON(100),
        OFF(0);

        double rpm;

        private FeederState(double rpm) {
            this.rpm = rpm;
        }
    }
}

