package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.LoggedTrapezoidProfile;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.TurretConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

public class TurretSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    private CANcoder encoder = new CANcoder(ENCODER_ID, CAN_BUS);
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private State targetState = new State();

    private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
    
    private TurretClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("turret").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("setpoint", Rotation2d.struct).publish();
    
    public TurretSubsystem() {

        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
            motorConfigurator.apply(PID_CONFIG);
            motorConfigurator.apply(CURRENT_LIMITS_CONFIG);
            motorConfigurator.apply(FEEDBACK_CONFIG);
            motorConfigurator.apply(MOTOR_OUTPUT_CONFIG); 

        CANcoderConfiguration cancoderConfigurator = new CANcoderConfiguration();
            cancoderConfigurator.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            cancoderConfigurator.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
            cancoderConfigurator.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
            encoder.getConfigurator().apply(cancoderConfigurator);
        
        currentClamp = TurretClamp.UNRESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        setMechanismAngle(Rotation2d.fromDegrees(getEncoderPosition().getDegrees()));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> targetState.position, (setpoint) -> setSetpoint(Rotation2d.fromDegrees(setpoint)));

        SmartDashboard.putData("Turret 0", setSetpointCommand(Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Turret 180", setSetpointCommand(Rotation2d.fromDegrees(180)));
        SmartDashboard.putData("Turret Motor", motor);
    }

    //#region Main Functionality
 
    @Override
    public void periodic(){
        targetState.position = MathUtil.clamp(
            targetState.position, 
            minAngle.getRotations(), 
            maxAngle.getRotations()
        );

        if (targetState.position <= minAngle.getRotations() || targetState.position >= maxAngle.getRotations()) {
            targetState.velocity = 0;
        }

        positionVoltage.withEnableFOC(ENABLE_FOC)
            .withPosition(targetState.position)
            .withVelocity(targetState.velocity);
            
        motor.setControl(positionVoltage);

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        positionPublisher.accept(getPosition());
        setpointPublisher.accept(Rotation2d.fromRotations(targetState.position));
    }

    public Rotation2d getPosition() {
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public Rotation2d getEncoderPosition() {
        double position = encoder.getAbsolutePosition().getValue().in(Rotations) * ENCODER_MECHANISM_RATIO;
        return Rotation2d.fromRotations(position);
    }

    public Rotation2d getCurrentSetpoint() {
        return Rotation2d.fromRotations(targetState.position);
    }

    public TurretClamp getClamp() {
        return currentClamp;
    }
    
    public void setSetpoint(Rotation2d setpoint){
        targetState.position = setpoint.getRotations();
        targetState.velocity = 0.0;
    }

    public void setComplexSetpoint(Rotation2d setPoint, AngularVelocity velocity){
        targetState.position = setPoint.getRotations();
        targetState.velocity = velocity.in(RotationsPerSecond);
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
        setSetpoint(angle);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setClampCommand(TurretClamp newClamp) {
        return this.runOnce(() -> setClamp(newClamp));
    }

    //#endregion

    public enum TurretClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(-145), Rotation2d.fromDegrees(225));

        public Rotation2d minAngle;
        public Rotation2d maxAngle;

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
