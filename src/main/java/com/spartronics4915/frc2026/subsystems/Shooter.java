package com.spartronics4915.frc2026.subsystems;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private TalonFX mainShooterMotor;
    private NeutralModeValue neutralMode;
    private TalonFX followerShooterMotor;

    private double currentSetSpeed;
    private State currentState;
    private TrapezoidProfile trapProfile;
    private SimpleMotorFeedforward FFCalculator;

    private DoublePublisher motorSpeed = 
    NetworkTableInstance.getDefault().getDoubleTopic("Actual shooter motor speed").publish();
    private DoublePublisher motorTargetSpeed = 
    NetworkTableInstance.getDefault().getDoubleTopic("Target shooter motor speed").publish();
    
        
        public Shooter () {

            trapProfile = new TrapezoidProfile(
	            new Constraints(Constants.ShooterConstants.MaxVelocity, Constants.ShooterConstants.MaxAcceleration)
            );

            currentState = new State(0, 0);

            mainShooterMotorInitializer();
            followerShooterMotorInitializer();

            FFCalculator = new SimpleMotorFeedforward(
                Constants.ShooterConstants.S,
                Constants.ShooterConstants.V,
                Constants.ShooterConstants.A
            );
            
        }

        public void mainShooterMotorInitializer() {
            mainShooterMotor = new TalonFX(Constants.ShooterConstants.mainShooterMotorID); 
            TalonFXConfigurator configForMainShooterMotor = mainShooterMotor.getConfigurator();
            configForMainShooterMotor.apply(new SlotConfigs()
                .withKP(Constants.ShooterConstants.MainP)
                .withKI(Constants.ShooterConstants.MainI)
                .withKD(Constants.ShooterConstants.MainD)
            );
            configForMainShooterMotor.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.ShooterConstants.SupplyCurrentLimitEnabled)
                .withSupplyCurrentLimit(Constants.ShooterConstants.SupplyCurrentLimit) 
                .withSupplyCurrentLowerLimit(Constants.ShooterConstants.SupplyCurrentLowerLimit)
                .withSupplyCurrentLowerTime(Constants.ShooterConstants.SupplyCurrentLowerTime)
            );
            configForMainShooterMotor.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterConstants.SensorToMechanismRatio)
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
        }

        public void followerShooterMotorInitializer() {
            followerShooterMotor = new TalonFX(Constants.ShooterConstants.followerShooterMotorID); 
            TalonFXConfigurator configForFollowerShooterMotor = followerShooterMotor.getConfigurator();
            configForFollowerShooterMotor.apply(new SlotConfigs()
                .withKP(Constants.ShooterConstants.FollowerP)
                .withKI(Constants.ShooterConstants.FollowerI)
                .withKD(Constants.ShooterConstants.FollowerD)
            );
            configForFollowerShooterMotor.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.ShooterConstants.SupplyCurrentLimitEnabled)
                .withSupplyCurrentLimit(Constants.ShooterConstants.SupplyCurrentLimit) 
                .withSupplyCurrentLowerLimit(Constants.ShooterConstants.SupplyCurrentLowerLimit)
                .withSupplyCurrentLowerTime(Constants.ShooterConstants.SupplyCurrentLowerTime)
            );
            configForFollowerShooterMotor.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterConstants.SensorToMechanismRatio)
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
        }
        
        public void setSpeed(double zeroToOne){
            currentSetSpeed = zeroToOne * Constants.ShooterConstants.maxSpeed;
        }

        public void setExactSpeed(double newSpeed){
            currentSetSpeed = newSpeed;
        }

        
        public AngularVelocity getSpeed(){
            return RPM.of(mainShooterMotor.getVelocity().getValue().in(RPM));
        }

        @Override
        public void periodic() {

            currentSetSpeed = MathUtil.clamp(
                currentSetSpeed,
                Constants.ShooterConstants.minSpeed,
                Constants.ShooterConstants.maxSpeed
            );

            
            motorSpeed.accept(currentState.position);  
            motorTargetSpeed.accept(currentSetSpeed);   
            currentState = trapProfile.calculate(
                Constants.ShooterConstants.deltaTime,  
                currentState, 
                new State(currentSetSpeed, 0)
            );

            VelocityVoltage request = new VelocityVoltage(
                currentState.position
            ).withFeedForward(
                FFCalculator.calculateWithVelocities(
                    currentState.position, 
                    currentSetSpeed
                )
            );

            mainShooterMotor.setControl(request);

        }

    public Command setSpeedCommand(double newSpeed) {
        return Commands.runOnce(() -> setSpeed(newSpeed));
    }

    public Command setExactSpeedCommand(double newSpeed) {
        return Commands.runOnce(() -> setExactSpeed(newSpeed));
    }
    
}  


