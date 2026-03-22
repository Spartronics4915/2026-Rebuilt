package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.FeederConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

public class FeederSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    
    private double currentSetpoint;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public FeederSubsystem() {
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);
        
        setState(FeederState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Feeder On", setStateCommand(FeederState.FORWARD));
        SmartDashboard.putData("Feeder Off", setStateCommand(FeederState.OFF));
        SmartDashboard.putData("Feeder Motor", motor);
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        if (currentSetpoint != 0) {
            velocityVoltage.withEnableFOC(ENABLE_FOC).Velocity = currentSetpoint;
            motor.setControl(velocityVoltage);
        } else {
            motor.setControl(new VoltageOut(0.0));
        }

        appliedOutPublisher.accept(motor.getCachedDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPM());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPM() {
        return motor.getCachedVelocity().getValueAsDouble();
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(FeederState state) {
        setSetpoint(state.rps);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(FeederState state){
        return setSetpointCommand(state.rps);
    }

    public enum FeederState{
        FORWARD(32.0),
        REVERSE(-32.0),
        OFF(0);

        double rps;

        private FeederState(double rps) {
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(FeederState.OFF);
    }
}

