package com.spartronics4915.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    TalonFX indexerMotor = new TalonFX(Constants.SpindexerConstants.SPINDEXER_MOTOR_ID);
    TalonFXConfigurator indexerConfigurator = indexerMotor.getConfigurator();

    State spindexerState = new State(Constants.SpindexerConstants.SPINDEXER_POSITION, Constants.SpindexerConstants.SPINDEXER_VELOCITY);
    double setPoint = (0.0);

    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new Constraints(Constants.SpindexerConstants.SPINDEXER_MAX_VELOCITY, Constants.SpindexerConstants.SPINDEXER_MAX_ACCELERATION)
    );

    DoublePublisher spindexerVoltage = NetworkTableInstance.getDefault().getDoubleTopic("Spindexer Voltage").publish();
    DoublePublisher spindexerRPM = NetworkTableInstance.getDefault().getDoubleTopic("Spindexer RPM").publish();

    //#region Main Functionality

    public void spindexer() {
        SmartDashboard.putData("Indexer On", setStateCommand(IndexerState.ON));
        SmartDashboard.putData("Indexer Off", setStateCommand(IndexerState.OFF));

        indexerConfigurator.apply( new SlotConfigs()
            .withKP(Constants.SpindexerConstants.SPINDEXER_P)
            .withKI(Constants.SpindexerConstants.SPINDEXER_I)
            .withKD(Constants.SpindexerConstants.SPINDEXER_D)
        );
        indexerConfigurator.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.SpindexerConstants.SPINDEXER_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.SpindexerConstants.SPINDEXER_LOWER_CURRENT_LIMIT)
            .withSupplyCurrentLowerTime(Constants.SpindexerConstants.SPINDEXER_LOWER_CURRENT_TIME)
            .withSupplyCurrentLimitEnable(Constants.SpindexerConstants.SPINDEXER_CURRENT_LIMIT_ENABLE)
        );
        indexerConfigurator.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.SpindexerConstants.SPINDEXER_SENSOR_TO_MECHANISM_RATIO)
        );
    }

    @Override
    public void periodic() {
        setPoint = MathUtil.clamp(
            setPoint, 
            Constants.SpindexerConstants.SPINDEXER_MAX_VELOCITY, 
            Constants.SpindexerConstants.SPINDEXER_MIN_VELOCITY
        );

        VelocityVoltage request = new VelocityVoltage(setPoint);

        indexerMotor.setControl(request);
        spindexerVoltage.accept(this.getAppliedVoltage());
        spindexerRPM.accept(this.getCurrentRPM());
    }

    public double getAppliedVoltage(){
        return indexerMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getCurrentRPM(){
        return indexerMotor.getAcceleration().getValueAsDouble();
    }

    public void setSetpoint(double newSetpoint){
        setPoint = newSetpoint;
    }

    //#endregion
    
    //#region Commands

    public Command setSetpointCommand(double newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setStateCommand(IndexerState state){
        return setSetpointCommand(state.rpm);
    }

    //#endregion

    public enum IndexerState {
        ON(100.0),
        OFF(0.0);

        public double rpm;
        private IndexerState(double rpm) {
            this.rpm = rpm;
        }
    }

}
