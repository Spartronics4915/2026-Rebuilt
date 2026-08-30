package com.spartronics4915.frc2026.subsystems.mechanisms;

import static com.spartronics4915.frc2026.Constants.IntakeConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Intake");

    // Enable FOC control and Switch to Velocity Voltage

    private LoggedTalonFX leadMotor = new LoggedTalonFX(LEAD_MOTOR_ID, CAN_BUS);
    private final StatusSignal<AngularVelocity> velocitySignal = leadMotor.getVelocity(false);
    private final StatusSignal<Double> dutyCycleSignal = leadMotor.getDutyCycle(false);
    private final BaseStatusSignal[] telemetrySignals = {velocitySignal, dutyCycleSignal};

    private double currentSetpoint;
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private double velocityRps;
    private double profileSetpointRps;

    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VoltageOut stopRequest = new VoltageOut(0.0);

    public IntakeSubsystem() {
        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        ModeSwitchHandler.EnableModeSwitchHandler(this);

        leadMotor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Intake On", setStateCommand(IntakeState.INTAKE));
        SmartDashboard.putData("Intake Off", setStateCommand(IntakeState.OFF));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(telemetrySignals);

        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        if (currentSetpoint != 0) {
            velocityTorqueRequest.Velocity = currentSetpoint;
            leadMotor.setControl(velocityTorqueRequest);
        } else {
            leadMotor.setControl(stopRequest);
        }

        appliedDutyCycle = dutyCycleSignal.getValueAsDouble();
        velocityRps = velocitySignal.getValueAsDouble();
        profileSetpointRps = currentSetpoint;
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("VelocityRps", velocityRps);
        LOG.critical.log("SetpointRps", currentSetpoint);
        LOG.critical.log("ProfileSetpointRps", profileSetpointRps);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
    }

    public double getCurrentRPS() {
        return velocityRps;
    }

    public double getAppliedVoltage() {
        return leadMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getSetpoint() {
        return currentSetpoint;
    }

    public void setSetpoint(double newSetpoint){
        currentSetpoint = newSetpoint;
    }

    public void setState(IntakeState newState) {
        setSetpoint(newState.rps);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(IntakeState state){
        return setSetpointCommand(state.rps);
    }

    public enum IntakeState {
        INTAKE(22),
        OUTTAKE(-22),
        OFF(0);

        double rps;

        private IntakeState(double rps) {
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(IntakeState.OFF);
    }
    
}
