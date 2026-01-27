package com.spartronics4915.frc2026.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2026.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    TalonFX primaryClimbMotor = new TalonFX(Constants.ClimberConstants.PRIMARY_CLIMB_MOTOR_ID);
    TalonFXConfigurator primaryClimbConfigurator = primaryClimbMotor.getConfigurator();

    DoublePublisher climberSetpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber Setpoint").publish();
    DoublePublisher climberRequestedPosPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Climber ").publish();

    public ClimberSubsystem() {
        applyMotorConfigs(primaryClimbConfigurator);

    }

    private void applyMotorConfigs(TalonFXConfigurator config) {
        config.apply(new SlotConfigs()
                .withKP(Constants.ClimberConstants.CLIMBER_P)
                .withKI(Constants.ClimberConstants.CLIMBER_I)
                .withKD(Constants.ClimberConstants.CLIMBER_D));
        config.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.ClimberConstants.CURRENT_LIMIT_ENABLED)
                .withSupplyCurrentLimit(Constants.ClimberConstants.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLowerLimit(Constants.ClimberConstants.CURRENT_LOWER_LIMIT)
                .withSupplyCurrentLowerTime(Constants.ClimberConstants.CURRENT_LOWER_TIME));
        config.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ClimberConstants.SENSOR_TO_MECHANISM_RATIO));

    }

    TrapezoidProfile trapProfile = new TrapezoidProfile(
            new Constraints(Constants.ClimberConstants.MAX_VELOCITY, Constants.ClimberConstants.MAX_ACCELERATION));
    double position = getPosition();
    State currentState = new State(position, 0);
    double currentSetPoint = position;
    ElevatorFeedforward FFCalculator = new ElevatorFeedforward(
            Constants.ClimberConstants.CLIMBER_S,
            Constants.ClimberConstants.CLIMBER_J,
            Constants.ClimberConstants.CLIMBER_V,
            Constants.ClimberConstants.CLIMBER_A);

    @Override
    public void periodic() {
        currentSetPoint = MathUtil.clamp(
                currentSetPoint,
                Constants.ClimberConstants.MIN_HIGHT,
                Constants.ClimberConstants.MAX_HIGHT);
        currentState = trapProfile.calculate(
                Constants.ClimberConstants.DeltaTime,
                currentState,
                new State(currentSetPoint, 0));
        PositionVoltage request = new PositionVoltage(currentState.position)
                .withFeedForward(
                        FFCalculator.calculate(currentState.position, currentState.velocity));

        primaryClimbMotor.setControl(request);
        publishData();
    }

    private void publishData() {
        climberSetpointPublisher.accept(currentSetPoint);
    }

    public Command setPrimaryClimber(double input) {
        return this.runOnce(() -> currentSetPoint = input);
    }

    public Command incrementPrimaryClimber(double input) {
        return this.runOnce(() -> currentSetPoint += input);
    }

    public double getPosition() {
        return primaryClimbMotor.getRotorPosition().getValueAsDouble();
    }

}
