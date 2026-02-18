package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.FeederConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

public class FeederSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    private TalonFX motor;

    private SlewRateLimiter RPSLimiter = new SlewRateLimiter(MAX_ACCELERATION);
    private double currentSetpoint;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("rps").publish();
    private final DoublePublisher desiredStatePublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("desired State").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public FeederSubsystem() {
        motor = new TalonFX(MOTOR_ID, CAN_BUS);
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
        
        setState(FeederState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("Feeder On", setStateCommand(FeederState.ON));
        SmartDashboard.putData("Feeder Off", setStateCommand(FeederState.OFF));
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        double limitedSetpoint;

        if (currentSetpoint != 0) {
            limitedSetpoint = RPSLimiter.calculate(currentSetpoint);
            VelocityVoltage request = new VelocityVoltage(limitedSetpoint);
            motor.setControl(request);
        } else {
            limitedSetpoint = 0;
            RPSLimiter.reset(0);
            VoltageOut request = new VoltageOut(0.0);
            motor.setControl(request);
        }

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPM());
        desiredStatePublisher.accept(limitedSetpoint);
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble();
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
        ON(40),
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

