package com.spartronics4915.frc2026.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private TalonFX mainShooterMotor;
    private NeutralModeValue neutralMode;
    private TalonFX followerShooterMotor;

    private double currentSetPoint;
    private State currentState;
    private TrapezoidProfile trapProfile;
    private SimpleMotorFeedforward FFCalculator;
    
    
        
        public Shooter () {
            //Main motor-----------------------------------------------------------------------------
            mainShooterMotor = new TalonFX(Constants.ShooterConstants.mainShooterMotorID); 
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

            if (Constants.ShooterConstants.motorCoast) {
                neutralMode = NeutralModeValue.Coast;
            } else {neutralMode = NeutralModeValue.Brake;}
            mainShooterMotor.setNeutralMode(neutralMode);


            //follower motor-----------------------------------------------------------------------------
            followerShooterMotor = new TalonFX(Constants.ShooterConstants.followerShooterMotorID); 
            TalonFXConfigurator configForFollowerShooterMotor = followerShooterMotor.getConfigurator();
            configForFollowerShooterMotor.apply(new SlotConfigs()
                .withKP(Constants.ShooterConstants.FollowerP)
                .withKI(Constants.ShooterConstants.FollowerI)
                .withKD(Constants.ShooterConstants.FollowerD)
            );
            configForFollowerShooterMotor.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(1.0)
            );
            configForFollowerShooterMotor.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(1)
            );
            MotorOutputConfigs followerShooterMotorOutputConfigs = new MotorOutputConfigs();
            if (!Constants.ShooterConstants.motorTurnsClockWise) {
                followerShooterMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            } else {followerShooterMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;}
            configForFollowerShooterMotor.apply(followerShooterMotorOutputConfigs);

            if (Constants.ShooterConstants.motorCoast) {
                neutralMode = NeutralModeValue.Coast;
            } else {neutralMode = NeutralModeValue.Brake;}
            followerShooterMotor.setNeutralMode(neutralMode);
            
            followerShooterMotor.setControl(new StrictFollower(Constants.ShooterConstants.mainShooterMotorID));

            FFCalculator = new SimpleMotorFeedforward(
                Constants.ShooterConstants.S,
                Constants.ShooterConstants.V,
                Constants.ShooterConstants.A);
            
        }
        
        public AngularVelocity getSpeed(){
            return RPM.of(mainShooterMotor.getVelocity().getValue().in(RPM));
        }

        @Override
        public void periodic() {
            currentSetPoint = MathUtil.clamp(
                currentSetPoint,
                Constants.ShooterConstants.minSpeed,
                Constants.ShooterConstants.maxSpeed
            );

            currentState = trapProfile.calculate(
                0.05, 
                currentState, 
                new State(currentSetPoint, 0)
            );

            VelocityVoltage request = new VelocityVoltage(
                currentState.position
            ).withFeedForward(
                FFCalculator.calculate(
                    currentState.position, 
                    currentState.velocity
                )
            );

                

            mainShooterMotor.setControl(request);

                

        }
    
    
}  


