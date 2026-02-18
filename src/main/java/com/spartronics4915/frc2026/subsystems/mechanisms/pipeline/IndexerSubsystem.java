package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.IndexerConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

public class IndexerSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    private TalonFX motor;
    
    private SlewRateLimiter RPSLimiter = new SlewRateLimiter(MAX_ACCELERATION);

    private double currentSetpoint;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("rps").publish();
    private final DoublePublisher desiredStatePublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("desired State").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("setpoint").publish();

    private final StructPublisher<Pose3d> componentPosePublisher = NetworkTableInstance.getDefault().getTable("indexer").getStructTopic("Indexer Component", Pose3d.struct).publish();

    //#region Main Functionality

    public IndexerSubsystem() {
        motor = new TalonFX(MOTOR_ID, CAN_BUS);
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(OUTPUT_CONFIG);

        setState(IndexerState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);
        
        SmartDashboard.putData("Indexer On", setStateCommand(IndexerState.ON));
        SmartDashboard.putData("Indexer Off", setStateCommand(IndexerState.OFF));
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

    public void setState(IndexerState state) {
        setSetpoint(state.rps);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(IndexerState state){
        return setSetpointCommand(state.rps);
    }

    public enum IndexerState {
        ON(20.0),
        OFF(0.0);

        public double rps;
        private IndexerState(double rps) {
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(IndexerState.OFF);
    }

}

