package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import static com.spartronics4915.frc2026.Constants.IndexerConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class IndexerSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Indexer");
    
    // Enable FOC control and Switch to Velocity Voltage

    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    private final StatusSignal<AngularVelocity> velocitySignal = motor.getVelocity(false);
    private final StatusSignal<Double> dutyCycleSignal = motor.getDutyCycle(false);
    private final BaseStatusSignal[] telemetrySignals = {velocitySignal, dutyCycleSignal};

    private double currentSetpoint;
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private double velocityRps;
    private double profileSetpointRps;
    private Pose3d mechanismPose = new Pose3d();
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);
    private final VoltageOut stopRequest = new VoltageOut(0.0);
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(50);
    
    private double indexerAngle = 0.0; // Tracks cumulative rotation angle in radians

    public IndexerSubsystem() {
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        setState(IndexerState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);
        
        SmartDashboard.putData("Indexer On", setStateCommand(IndexerState.FORWARD));
        SmartDashboard.putData("Indexer Off", setStateCommand(IndexerState.OFF));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(telemetrySignals);

        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        double limitedSetpoint = slewRateLimiter.calculate(currentSetpoint);

        if (limitedSetpoint != 0) {
            velocityTorqueRequest.Velocity = limitedSetpoint;
            motor.setControl(velocityTorqueRequest);
        } else {
            motor.setControl(stopRequest);
        }
        
        double velocityRps = Robot.isReal()
            ? velocitySignal.getValueAsDouble()
            : getCurrentSetpoint();
        indexerAngle += velocityRps * 2 * Math.PI * 0.02;
        appliedDutyCycle = dutyCycleSignal.getValueAsDouble();
        this.velocityRps = velocityRps;
        profileSetpointRps = currentSetpoint;
        mechanismPose = new Pose3d(
                0.0472, -0.0002, 0.0619, 
                new Rotation3d(0, 0, indexerAngle)
            );
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("VelocityRps", velocityRps);
        LOG.critical.log("SetpointRps", currentSetpoint);
        LOG.critical.log("ProfileSetpointRps", profileSetpointRps);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
        LOG.debug.log("MechanismPose", mechanismPose);
    }

    public double getCurrentRPS() {
        return Robot.isReal() ? velocityRps : getCurrentSetpoint();
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(IndexerState state) {
        setSetpoint(state.rps);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(IndexerState state){
        return setSetpointCommand(state.rps);
    }

    public enum IndexerState {
        FORWARD(22.0),
        REVERSE(-22.0),
        OFF(0.0);

        public double rps;
        private IndexerState(double rps) { 
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(IndexerState.OFF);
    }

}

