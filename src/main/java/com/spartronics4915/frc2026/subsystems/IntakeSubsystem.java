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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    //Initialize motor
    TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    TalonFXConfigurator intakeMotorConfig = intakeMotor.getConfigurator();

    
    //Configure that crap so the intake wont murder the insides of the robot
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
    public AngularVelocity getSpeed(){
        return RPM.of(intakeMotor.getVelocity().getValue().in(RPM));
    }
}
