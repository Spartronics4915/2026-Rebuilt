package com.spartronics4915.frc2026.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    //Initialize motor
    TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();

    
    //Constructor
    public IntakeSubsystem() {
        intakeMotorConfig.apply(new SlotConfigs()
                .withKP(Constants.IntakeConstants.INTAKE_P)
                .withKI(Constants.IntakeConstants.INTAKE_I)
                .withKD(Constants.IntakeConstants.INTAKE_D)
        );
        intakeMotorConfig.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_ENABLE)
                .withSupplyCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT)
                .withSupplyCurrentLowerLimit(Constants.IntakeConstants.INTAKE_CURRENT_LOWER_LIMIT)
                .withSupplyCurrentLowerTime(Constants.IntakeConstants.INTAKE_CURRENT_LOWER_TIME)
        );
        intakeMotorConfig.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.IntakeConstants.INTAKE_SENSOR_TO_MECH_RATIO)
        );
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        intakeMotorConfig.apply(motorOutputConfigs);
    }

    //Value returns
    public AngularVelocity getSpeed(){
        //Returns RPM of the intakeMotor
        return RPM.of(intakeMotor.getVelocity().getValue().in(RPM));
    }
    public Voltage getIntakeVoltage(){
        //Returns Voltage of the intakeMotor
        return (Voltage) intakeMotor.getMotorVoltage();
    }
    public boolean isSpinning() {
        //Returns if the motor is spinning
        if (this.getSpeed().isEquivalent(RPM.of(0))) {
            return false;
        }
        else {
            return true;
        }
    }
    public boolean hasPower() {
        //Returns if the motor has voltage
        if (this.getIntakeVoltage().isEquivalent(Voltage.ofBaseUnits(0.0, null))) {
            return false;
        }
        else {
            return true;
        }
    }

    //Makes the motor do motor things
    public void enableIntakeMotor() {
        //runs the motor at the speed constant
        intakeMotor.set(Constants.IntakeConstants.INTAKE_MOTOR_SPEED);
    }
    public void disableIntakeMotor() {
        //stops the motor
        intakeMotor.set(0.0);
    }

    //Logging
    public void motorLog() {
        //Prints out RPM and Voltage when called
        System.out.println("Current RPM of IntakeMotor: " + this.getSpeed());
        System.out.println("Current Voltage of IntakeMotor: " + this.getIntakeVoltage());
    }
}