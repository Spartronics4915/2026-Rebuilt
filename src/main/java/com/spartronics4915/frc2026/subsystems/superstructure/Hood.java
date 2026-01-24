package com.spartronics4915.frc2026.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.spartronics4915.frc2026.Constants;

public class Hood extends SubsystemBase {
    TalonFX robotsonTheThird = new TalonFX(21);

    public Hood() {
        TalonFXConfigurator motorConfig = robotsonTheThird.getConfigurator();
        motorConfig.apply(new SlotConfigs()
                .withKP(Constants.HoodConstants.P)
                .withKI(Constants.HoodConstants.I)
                .withKD(Constants.HoodConstants.D));
        motorConfig.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLowerLimit(60)
                .withSupplyCurrentLowerTime(1.0));
        motorConfig.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(1 / 8));
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.apply(motorOutputConfigs);
        
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(robotsonTheThird.getPosition()
                .getValue().in(Degrees));
    }

    public AngularVelocity getSpeed() {
        return RPM.of(robotsonTheThird.getVelocity().getValue().in(RPM));
    }
    

}
