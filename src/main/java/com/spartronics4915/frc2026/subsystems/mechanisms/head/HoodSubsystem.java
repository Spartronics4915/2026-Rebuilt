package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.spartronics4915.frc2026.util.TimeVarianceAuthority;

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

import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;
import static com.spartronics4915.frc2026.Constants.HoodConstants.*;

public class HoodSubsystem extends SubsystemBase implements ModeSwitchInterface {

    TalonFX motor = new TalonFX(MOTOR_ID);
    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private Rotation2d currentSetpoint = Rotation2d.fromDegrees(0);
    private State currentState = new State();

    private HoodClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("hood").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("hood").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> desiredStatePublisher = NetworkTableInstance.getDefault().getTable("hood").getStructTopic("desiredState", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("hood").getStructTopic("setpoint", Rotation2d.struct).publish();
    
    public HoodSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
            motorConfig.apply(motorOutputConfigs);

        currentClamp = HoodClamp.RESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        setMechanismAngle(Rotation2d.fromDegrees(0));
        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    //#region Main Functionality

    @Override
    public void periodic(){
        currentSetpoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                currentSetpoint.getRotations(), 
                minAngle.getRotations(), 
                maxAngle.getRotations()
            )
        );

        currentState = trapezoidProfile.calculate(
            dtCalc.update(), 
            currentState, 
            new State(currentSetpoint.getRotations(), 0.0)
        );

        PositionVoltage request = new PositionVoltage(currentState.position);
            motor.setControl(request);

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
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

    public void setClamp(HoodClamp clamp){
        currentClamp = clamp;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        resetMechanism(angle);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(Rotation2d angle){
        currentSetpoint = angle;
        currentState = new State(angle.getRotations(), 0.0);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }
 
    public enum HoodClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

        Rotation2d minAngle;
        Rotation2d maxAngle;

        private HoodClamp(Rotation2d minAngle, Rotation2d maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
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
