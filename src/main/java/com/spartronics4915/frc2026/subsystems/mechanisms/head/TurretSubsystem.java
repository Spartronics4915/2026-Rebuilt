package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.TimeVarianceAuthority;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
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

import static com.spartronics4915.frc2026.Constants.TurretConstants.*;

public class TurretSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private TalonFX motor = new TalonFX(MOTOR_ID);
    private CANcoder encoder = new CANcoder(ENCODER_ID);
    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private Rotation2d currentSetpoint = Rotation2d.fromDegrees(0);
    private State currentState = new State();
    
    private TurretClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("turret").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> desiredStatePublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("desiredState", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("setpoint", Rotation2d.struct).publish();

    private final StructPublisher<Pose3d> componentPosePublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("Turret Component", Pose3d.struct).publish();
    
    public TurretSubsystem(){
        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
            motorConfigurator.apply(PID_CONFIG);
            motorConfigurator.apply(CURRENT_LIMITS_CONFIG);
            motorConfigurator.apply(FEEDBACK_CONFIG);

        CANcoderConfiguration cancoderConfigurator = new CANcoderConfiguration();
            cancoderConfigurator.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            cancoderConfigurator.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
            cancoderConfigurator.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
        
        currentClamp = TurretClamp.RESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        setMechanismAngle(Rotation2d.fromDegrees(getEncoderPosition().getDegrees()));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("Turret 0", setSetpointCommand(Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Turret 180", setSetpointCommand(Rotation2d.fromDegrees(180)));
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

    public Rotation2d getEncoderPosition() {
        double position = encoder.getPosition().getValue().in(Rotations) * ENCODER_MECHANISM_RATIO;
        return Rotation2d.fromRotations(position);
    }
    
    public void setSetpoint(Rotation2d setpoint){
        currentSetpoint = setpoint;
    }

    public void setClamp(TurretClamp clamp){
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

    //#endregion

    public enum TurretClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

        Rotation2d minAngle;
        Rotation2d maxAngle;

        private TurretClamp(Rotation2d minAngle, Rotation2d maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}
