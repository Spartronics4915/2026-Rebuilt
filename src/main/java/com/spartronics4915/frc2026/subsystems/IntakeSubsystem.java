package com.spartronics4915.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // Initialize motor
    TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();

    TrapezoidProfile trapProfile = new TrapezoidProfile(
            new Constraints(Constants.IntakeConstants.INTAKE_MAX_VELOCITY, Constants.IntakeConstants.INTAKE_MAX_ACCELERATION));

    State currentState = new State(Constants.IntakeConstants.INTAKE_POSITION, Constants.IntakeConstants.INTAKE_VELOCITY);

    double currentSetPoint = 0.0;

    DoublePublisher intakeRPMPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Current RPM of IntakeMotor: ").publish();
    DoublePublisher intakeVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Current Voltage of IntakeMotor: ").publish();

    // Constructor
    public IntakeSubsystem() {
        SmartDashboard.putBoolean("Active", false);
        intakeMotorConfig.apply(new SlotConfigs()
                .withKP(Constants.IntakeConstants.INTAKE_P)
                .withKI(Constants.IntakeConstants.INTAKE_I)
                .withKD(Constants.IntakeConstants.INTAKE_D));
        intakeMotorConfig.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_ENABLE)
                .withSupplyCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT)
                .withSupplyCurrentLowerLimit(Constants.IntakeConstants.INTAKE_CURRENT_LOWER_LIMIT)
                .withSupplyCurrentLowerTime(Constants.IntakeConstants.INTAKE_CURRENT_LOWER_TIME));
        intakeMotorConfig.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.IntakeConstants.INTAKE_SENSOR_TO_MECH_RATIO));
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfig.apply(motorOutputConfigs);
    }

    // Value returns
    public double getSpeed() {
        // Returns RPM of the intakeMotor
        return (intakeMotor.getVelocity().getValueAsDouble());
    }

    public double getIntakeVoltage() {
        // Returns Voltage of the intakeMotor
        return (intakeMotor.getMotorVoltage().getValueAsDouble());
    }

    public boolean isSpinning() {
        // Returns if the motor is spinning
        if (this.getSpeed() == 0.0) {
            return false;
        } else {
            return true;
        }
    }

    public boolean hasPower() {
        // Returns if the motor has voltage
        if (this.getIntakeVoltage() == 0.0) {
            return false;
        } else {
            return true;
        }
    }

    // Makes the motor do motor things
    public void enableIntakeMotor() {
        // runs the motor at the speed constant
       currentSetPoint = Constants.IntakeConstants.INTAKE_MOTOR_SPEED;
    }

    public void disableIntakeMotor() {
        // stops the motor
        currentSetPoint = 0;
    }

    @Override
    public void periodic() {
        // PID loop
        currentSetPoint = MathUtil.clamp(currentSetPoint, Constants.IntakeConstants.INTAKE_MINIMUM_VELOCITY,
                Constants.IntakeConstants.INTAKE_MAXIMUM_VELOCITY);

        // currentState = trapProfile.calculate(
        //     Constants.IntakeConstants.INTAKE_DT, 
        //     currentState, 
        //     new State(0, currentSetPoint));

        VelocityVoltage request = new VelocityVoltage(currentSetPoint);

        intakeMotor.setControl(request);

        // Prints out RPM and Voltage
        intakeRPMPublisher.accept(this.getSpeed());
        intakeVoltagePublisher.accept(this.getIntakeVoltage());
        if (SmartDashboard.getBoolean("Active", false)) {
            enableIntakeMotor();
        } else {
            disableIntakeMotor();
        }

    }
}