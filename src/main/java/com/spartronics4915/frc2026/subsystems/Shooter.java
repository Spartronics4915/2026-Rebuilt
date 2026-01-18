package com.spartronics4915.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX mainShooterMotor;

    
    
        
        public Shooter () {
            
            mainShooterMotor = new TalonFX(Constants.ShooterConstants.mainShooterMotorID); // motor IIIDDDDDDD
            TalonFXConfigurator configForMainShooterMotor = mainShooterMotor.getConfigurator();
            configForMainShooterMotor.apply(new SlotConfigs()
                .withKP(Constants.ShooterConstants.MainP)
                .withKI(Constants.ShooterConstants.MainI)
                .withKD(Constants.ShooterConstants.MainD)
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
            MotorOutputConfigs mainShooterMotorOutputConfigs = new MotorOutputConfigs();
            if (Constants.ShooterConstants.motorTurnsClockWise) {
                mainShooterMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            } else {mainShooterMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;}
            configForMainShooterMotor.apply(mainShooterMotorOutputConfigs);
        }
        
        public Rotation2d getAngle(){
            return Rotation2d.fromDegrees(mainShooterMotor.getPosition().getValue().in(Degrees));
        }
        
        public AngularVelocity getSpeed(){
            return RPM.of(mainShooterMotor.getVelocity().getValue().in(RPM));
        }
    
    
}  