package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import static com.spartronics4915.frc2026.Constants.HoodConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class HoodSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Hood");

    // May need to retune or switch to position + FOC

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    private final StatusSignal<Angle> motorPositionSignal = motor.getPosition(false);
    private final StatusSignal<Double> dutyCycleSignal = motor.getDutyCycle(false);
    private final BaseStatusSignal[] telemetrySignals = {
        motorPositionSignal,
        dutyCycleSignal
    };
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private State targetState = new State();
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private Rotation2d loggedPosition = Rotation2d.kZero;
    private Rotation2d loggedSetpoint = Rotation2d.kZero;
    private Rotation2d profileSetpoint = Rotation2d.kZero;
    private Rotation2d encoderPosition = Rotation2d.kZero;
    private Pose3d mechanismPose = new Pose3d();
    
    private TurretSubsystem turret;

    private final PositionTorqueCurrentFOC positionTorqueRequest = new PositionTorqueCurrentFOC(0.0);

    private HoodClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    public HoodSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);
            motorConfig.apply(MOTOR_OUTPUT_CONFIG);

        currentClamp = HoodClamp.UNRESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        setMechanismAngle(Rotation2d.fromDegrees(0));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> targetState.position, (setpoint) -> setSetpoint(Rotation2d.fromDegrees(setpoint)));

        SmartDashboard.putData("Hood Up", setSetpointCommand(Rotation2d.fromDegrees(19)));
        SmartDashboard.putData("Hood Down", setSetpointCommand(Rotation2d.fromDegrees(0)));
    }

    public void setTurretSubsystem(TurretSubsystem turretSubsystem) {
        this.turret = turretSubsystem;
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(telemetrySignals);

        targetState.position = MathUtil.clamp(
            targetState.position, 
            minAngle.getRotations(), 
            maxAngle.getRotations()
        );

        if (targetState.position <= minAngle.getRotations() || targetState.position >= maxAngle.getRotations()) {
            targetState.velocity = 0;
        }
        
        positionTorqueRequest
            .withPosition(targetState.position)
            .withVelocity(targetState.velocity);
            
        motor.setControl(positionTorqueRequest);

        Rotation2d position = Robot.isSimulation()
            ? Rotation2d.fromRotations(targetState.position)
            : Rotation2d.fromRotations(motorPositionSignal.getValueAsDouble());
        Rotation2d setpoint = Rotation2d.fromRotations(targetState.position);
        double turretAngle = (turret != null) ? turret.getPosition().getRadians() : 0.0;
        Rotation3d hoodRotation = new Rotation3d(-position.getRadians(), 0, turretAngle);

        appliedDutyCycle = dutyCycleSignal.getValueAsDouble();
        loggedPosition = position;
        loggedSetpoint = setpoint;
        profileSetpoint = setpoint;
        encoderPosition = position;
        mechanismPose = new Pose3d(-0.1181, -0.0972 - 0.0453, 0.4953 + position.getTan() * 0.0453, hoodRotation);
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("Position", loggedPosition, Rotation2d.struct);
        LOG.critical.log("Setpoint", loggedSetpoint, Rotation2d.struct);
        LOG.critical.log("ProfileSetpoint", profileSetpoint, Rotation2d.struct);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
        LOG.info.log("EncoderPosition", encoderPosition, Rotation2d.struct);
        LOG.debug.log("MechanismPose", mechanismPose);
    }

    public Rotation2d getPosition() {
        if (Robot.isSimulation()) {
            return Rotation2d.fromRotations(targetState.position);
        }
        return loggedPosition;
    }

    public Rotation2d getCurrentSetpoint() {
        return Rotation2d.fromRotations(targetState.position);
    }

    public void setSetpoint(Rotation2d setpoint){
        targetState.position = setpoint.getRotations();
        targetState.velocity = 0.0;
    }

    public void setComplexSetpoint(Rotation2d setPoint, AngularVelocity velocity){
        targetState.position = setPoint.getRotations();
        targetState.velocity = velocity.in(RotationsPerSecond);
    }

    public void setClamp(HoodClamp clamp){
        currentClamp = clamp;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        loggedPosition = angle;
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

    public Command setClampCommand(HoodClamp newClamp) {
        return this.runOnce(() -> setClamp(newClamp));
    }
 
    public enum HoodClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(40));

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

    //  H   H  OOO   OOO  DDDD      CCC   OOO  DDDD  EEEEE   //
    //  H   H O   O O   O D   D    C   C O   O D   D E       //
    //  H   H O   O O   O D   D    C     O   O D   D E       //
    //  HHHHH O   O O   O D   D    C     O   O D   D EEEE    //
    //  H   H O   O O   O D   D    C     O   O D   D E       //
    //  H   H O   O O   O D   D    C   C O   O D   D E       //
    //  H   H  OOO   OOO  DDDD      CCC   OOO  DDDD  EEEEE   //
