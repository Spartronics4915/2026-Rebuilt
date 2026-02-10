package com.spartronics4915.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.HoodConstants.*;

public class HoodSubsystem extends SubsystemBase {

    TalonFX motor = new TalonFX(HOOD_MOTOR_ID);
    
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(HOOD_MAX_VELOCITY, HOOD_MAX_ACCELERATION)
    );

    private Rotation2d currentSetpoint = Rotation2d.fromDegrees(0);
    private State currentState;

    DoublePublisher hoodAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood Angle").publish();
    DoublePublisher hoodVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood Voltage").publish();
    
    public HoodSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
        
        motorConfig.apply(new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D)
        );
        
        motorConfig.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(HOOD_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(HOOD_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(HOOD_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(HOOD_LOWER_TIME)
        );
        
        motorConfig.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(HOOD_SENSOR_MECHANISM_RATIO)
        );

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.apply(motorOutputConfigs);

        SmartDashboard.putData("Hood Up", setStateCommand(HoodState.UP));
        SmartDashboard.putData("Hood Down", setStateCommand(HoodState.DOWN));
    }

    //#region Main Functionality

    @Override
    public void periodic(){
    
        currentSetpoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                currentSetpoint.getRotations(), 
                MIN_ANGLE.getRotations(), 
                MAX_ANGLE.getRotations()
        ));

        currentState = trapProfile.calculate(
            Constants.HoodConstants.HOOD_DT, 
            currentState, 
            new State(currentSetpoint.getDegrees(), 0.0)
        );

        PositionVoltage request = new PositionVoltage(currentState.position);
        motor.setControl(request);

        hoodAnglePublisher.accept(motor.getPosition().getValueAsDouble());
        hoodVoltagePublisher.accept(motor.getMotorVoltage().getValueAsDouble());
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(
            motor.getPosition().getValue().in(Degrees)
        );
    }

    public void setSetpoint(Rotation2d setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(HoodState state){
        currentSetpoint = state.angle;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(HoodState state){
        return this.runOnce(() -> setSetpoint(state.angle));
    }
 
    public enum HoodState {
        DOWN(Rotation2d.fromDegrees(0)),
        UP(Rotation2d.fromDegrees(35)),
        MIDDLE(Rotation2d.fromDegrees(17.5));

        Rotation2d angle;

        private HoodState(Rotation2d angle) {
            this.angle = angle;
        }
    }

}
    // This is so awesome!

    //   H   H  OOO   OOO  DDDD       CCC   OOO  DDDD  EEEEE   //
    //   H   H O   O O   O D   D     C   C O   O D   D E       //
    //   H   H O   O O   O D   D     C     O   O D   D E       //
    //   HHHHH O   O O   O D   D     C     O   O D   D EEEE    //
    //   H   H O   O O   O D   D     C     O   O D   D E       //
    //   H   H O   O O   O D   D     C   C O   O D   D E       //
    //   H   H  OOO   OOO  DDDD       CCC   OOO  DDDD  EEEEE   //