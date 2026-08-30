package com.spartronics4915.frc2026.subsystems.mechanisms;

import static com.spartronics4915.frc2026.Constants.ClimberConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.LoggedTrapezoidProfile;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Climber");

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    private final StatusSignal<Angle> motorPositionSignal = motor.getPosition(false);
    private final StatusSignal<Double> dutyCycleSignal = motor.getDutyCycle(false);
    private final BaseStatusSignal[] telemetrySignals = {
        motorPositionSignal,
        dutyCycleSignal
    };
    LoggedTrapezoidProfile trapProfile = new LoggedTrapezoidProfile(
	    new Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private double currentSetpoint;
    private State currentState = new State();
    private final State goalState = new State();
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private double loggedPosition;
    private double profileSetpoint;

    private final PositionVoltage positionVoltage = new PositionVoltage(0.0);

    public ClimberSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);
            motorConfig.apply(MOTOR_OUTPUT_CONFIG);

        BaseStatusSignal.refreshAll(telemetrySignals);
        setMechanismPosition(motorPositionSignal.getValueAsDouble());
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addProfile(trapProfile);
        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Climber Climb", setStateCommand(ClimberState.JORBIT));
        SmartDashboard.putData("Climber Down", setStateCommand(ClimberState.DOWN));
    }

    @Override
    public void periodic(){
        BaseStatusSignal.refreshAll(telemetrySignals);
        goalState.position = currentSetpoint;
        goalState.velocity = 0.0;

        currentSetpoint = MathUtil.clamp(
            currentSetpoint, 
            MIN_HEIGHT, 
            MAX_HEIGHT
        );

        currentState = trapProfile.calculate(
            dtCalc.update(), 
            currentState, 
            goalState
        );

        positionVoltage.withEnableFOC(ENABLE_FOC).Position = currentState.position;
        motor.setControl(positionVoltage);

        appliedDutyCycle = dutyCycleSignal.getValueAsDouble();
        loggedPosition = motorPositionSignal.getValueAsDouble();
        profileSetpoint = currentState.position;
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("Position", loggedPosition);
        LOG.critical.log("Setpoint", currentSetpoint);
        LOG.critical.log("ProfileSetpoint", profileSetpoint);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
    }

    public double getPosition() {
        return loggedPosition;
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(ClimberState state){
        currentSetpoint = state.position;
    }

    private void setMechanismPosition(double position){
        motor.setPosition(position);
        loggedPosition = position;
        resetMechanism(position);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(double position){
        currentSetpoint = position;
        currentState = new State(position, 0.0);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setStateCommand(ClimberState state){
        return setSetpointCommand(state.position);
    }

    //#endregion
 
    public enum ClimberState {
        DOWN(0),
        JORBIT(2.6);

        double position;

        private ClimberState(double position) {
            this.position = position;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}
