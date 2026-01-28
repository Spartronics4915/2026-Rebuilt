package com.spartronics4915.frc2026.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.spartronics4915.frc2026.Constants;

public class Hood extends SubsystemBase {
    ArmFeedforward FFCalculator = new ArmFeedforward(Constants.HoodConstants.S,Constants.HoodConstants.G,Constants.HoodConstants.V,Constants.HoodConstants.A);
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(Constants.HoodConstants.MaxVelocity, Constants.HoodConstants.MaxAcceleration)
    );
    State currentState = new State(Constants.HoodConstants.position, Constants.HoodConstants.velocity){};
    double currentSetPoint = 0;
    
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
    @SuppressWarnings("unused")
    private void setVoltage(double volts){
        robotsonTheThird.setVoltage(volts);
    }

    @Override
    public void periodic(){
        currentSetPoint = MathUtil.clamp(
            currentSetPoint,
            Constants.HoodConstants.MIN,
            Constants.HoodConstants.MAX
        );

        currentState = trapProfile.calculate(
            Constants.HoodConstants.dt, 
            currentState, 
            new State(currentSetPoint, 0)
        );

        PositionVoltage request = new PositionVoltage(
            currentState.position
        ).withFeedForward(
            FFCalculator.calculate(
            currentState.position, 
            currentState.velocity
            )
        );

        robotsonTheThird.setControl(request);

    }

}