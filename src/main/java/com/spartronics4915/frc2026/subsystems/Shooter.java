package com.spartronics4915.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX mainShooterMotor;

    
    
        
        public Shooter () {
            mainShooterMotor = new TalonFX(100); // motor IIIDDDDDDD
            TalonFXConfigurator configForMainShooterMotor = mainShooterMotor.getConfigurator();
            configForMainShooterMotor.apply(new SlotConfigs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
            );
            configForMainShooterMotor.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(1.0)
            );
            configForMainShooterMotor.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(1)
            );
            MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            configForMainShooterMotor.apply(motorOutputConfigs);
        }
        
        public Rotation2d getAngle(){
            return Rotation2d.fromDegrees(mainShooterMotor.getPosition().getValue().in(Degrees));
        }
        
        public AngularVelocity getSpeed(){
            return RPM.of(mainShooterMotor.getVelocity().getValue().in(RPM));
        }
    
    
}  