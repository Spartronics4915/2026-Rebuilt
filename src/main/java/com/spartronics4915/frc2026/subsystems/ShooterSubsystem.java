package com.spartronics4915.frc2026.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX leadMotor;
    private TalonFX followerMotor;

    private double currentSetpoint;

    //#region Main Functionality

    public ShooterSubsystem() {
        this.currentSetpoint = 0.0;

        leadMotor = new TalonFX(LEAD_MOTOR_ID);
        leadMotor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);

        followerMotor = new TalonFX(FOLLOWER_MOTOR_ID);
            
        configurator = followerMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);

        followerMotor.setControl(new Follower(FOLLOWER_MOTOR_ID, null)); 
    }

    @Override
    public void periodic() {
        PositionVoltage request = new PositionVoltage(currentSetpoint);
        leadMotor.setControl(request);
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(ShooterState state) {
        setSetpoint(state.rpm);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(ShooterState state){
        return setSetpointCommand(state.rpm);
    }

    public enum ShooterState{
        ON(100),
        OFF(0);

        double rpm;

        private ShooterState(double rpm) {
            this.rpm = rpm;
        }
    }
}
