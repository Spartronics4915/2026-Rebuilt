package com.spartronics4915.frc2026.subsystems.mechanisms;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.PivotConstants.*;

public class PivotSubsystem extends SubsystemBase {

    TalonFX motor = new TalonFX(PIVOT_MOTOR_ID);
    
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    private Rotation2d currentSetpoint = Rotation2d.fromDegrees(0);
    private State currentState = new State();

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("Pivot").getDoubleTopic("Applied Out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("Pivot").getStructTopic("Position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> desiredStatePublisher = NetworkTableInstance.getDefault().getTable("Pivot").getStructTopic("Desired State", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("Pivot").getStructTopic("Setpoint", Rotation2d.struct).publish();
    
    public PivotSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motorConfig.apply(motorOutputConfigs);

        currentState = new State(0, 0.0);
        motor.setPosition(0);
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

        appliedOutPublisher.accept(motor.getMotorVoltage().getValue().in(Volts));
        positionPublisher.accept(getPosition());
        desiredStatePublisher.accept(Rotation2d.fromRotations(currentState.position));
        setpointPublisher.accept(currentSetpoint);
    }

    public Rotation2d getPosition() {
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public void setSetpoint(Rotation2d setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(PivotState state){
        currentSetpoint = state.angle;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(PivotState preset){
        return setSetpointCommand(preset.angle);
    }
 
    public enum PivotState {
        UP(Rotation2d.fromDegrees(0)),
        DOWN(Rotation2d.fromDegrees(0));

        Rotation2d angle;

        private PivotState(Rotation2d angle) {
            this.angle = angle;
        }
    }

}