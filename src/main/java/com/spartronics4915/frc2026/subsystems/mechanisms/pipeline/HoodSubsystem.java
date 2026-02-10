package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.HoodConstants.*;

public class HoodSubsystem extends SubsystemBase {

    TalonFX motor = new TalonFX(MOTOR_ID);
    
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    private Rotation2d currentSetpoint = Rotation2d.fromDegrees(0);
    private State currentState = new State();

    private final DoublePublisher appliedOutPub = NetworkTableInstance.getDefault().getTable("logHood").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPub = NetworkTableInstance.getDefault().getTable("logHood").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> desiredStatePub = NetworkTableInstance.getDefault().getTable("logHood").getStructTopic("desiredState", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointpub = NetworkTableInstance.getDefault().getTable("logHood").getStructTopic("setpointpub", Rotation2d.struct).publish();
    
    public HoodSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motorConfig.apply(motorOutputConfigs);

        currentState = new State(0, 0.0);
        motor.setPosition(0);

        SmartDashboard.putData("Hood Up", presetCommand(HoodState.UP));
        SmartDashboard.putData("Hood Middle", presetCommand(HoodState.MIDDLE));
        SmartDashboard.putData("Hood Down", presetCommand(HoodState.DOWN));
    }

    //#region Main Functionality

    @Override
    public void periodic(){
    
        currentSetpoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                currentSetpoint.getRotations(), 
                MIN_ANGLE.getRotations(), 
                MAX_ANGLE.getRotations()
            )
        );

        currentState = trapProfile.calculate(
            DELTA_TIME, 
            currentState, 
            new State(currentSetpoint.getRotations(), 0.0)
        );

        PositionVoltage request = new PositionVoltage(currentState.position);
        motor.setControl(request);

        appliedOutPub.accept(motor.getMotorVoltage().getValue().in(Volts));
        positionPub.accept(getPosition());
        desiredStatePub.accept(Rotation2d.fromRotations(currentState.position));
        setpointpub.accept(currentSetpoint);
    }

    public Rotation2d getPosition() {
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public void setSetpoint(Rotation2d setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(HoodState state){
        currentSetpoint = state.angle;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(HoodState preset){
        return setSetpointCommand(preset.angle);
    }
 
    public enum HoodState {
        DOWN(Rotation2d.fromDegrees(0)),
        UP(Rotation2d.fromDegrees(30)),
        MIDDLE(Rotation2d.fromDegrees(15));

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