package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import static edu.wpi.first.units.Units.Volts;

import static com.spartronics4915.frc2026.Constants.FeederConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import au.grapplerobotics.LaserCan;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class FeederSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private static final Scope LOG = Telemetry.scope("Mechanisms/Feeder");

    // Enable FOC control and Switch to Velocity Voltage
    
    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    
    private double currentSetpoint;
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private double velocityRps;
    private double profileSetpointRps;
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);

    private DoubleSupplier distanceToTargetSupplier = null;
    private boolean dynamicSpeedActive = false;

    private final TorqueCurrentFOC sysIdControl = new TorqueCurrentFOC(0.0);
    private boolean isCharacterizing = false;
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null, 
            null
        ),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> motor.setControl(sysIdControl.withOutput(volts.in(Volts))),
            (log) -> {
                log.motor("Feeder")
                    .voltage(Volts.of(motor.getTorqueCurrent().getValueAsDouble()))
                    .angularVelocity(motor.getVelocity().getValue())
                    .angularPosition(motor.getPosition().getValue())
                    .angularAcceleration(motor.getAcceleration().getValue());
            },
            this
        )
    );

    public FeederSubsystem() {
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);
        
        setState(FeederState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Feeder Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Feeder Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("Feeder Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Feeder Dynamic Reverse", sysIdDynamic(Direction.kReverse));

        SmartDashboard.putData("Feeder On", setStateCommand(FeederState.FORWARD));
        SmartDashboard.putData("Feeder Off", setStateCommand(FeederState.OFF));
    }
    
    @Override
    public void periodic() {
        // When dynamic speed is active, override the static setpoint with the
        // interpolated value from the distance→RPS lookup table.
        if (dynamicSpeedActive && distanceToTargetSupplier != null) {
            //currentSetpoint = feederSpeedMap.get(
            //    distanceToTargetSupplier.getAsDouble()
            //);
            currentSetpoint = 22.87887 / (1 + Math.pow(
                Math.E, 
                -(((0.928997 * distanceToTargetSupplier.getAsDouble()) - 1.56251)))
            );
        }

        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        if (!isCharacterizing) {
            if (currentSetpoint != 0) {
                velocityTorqueRequest.Velocity = currentSetpoint;
                motor.setControl(velocityTorqueRequest);
            } else {
                motor.setControl(new VoltageOut(0.0));
            }
        }

        appliedDutyCycle = motor.getDutyCycle().getValueAsDouble();
        velocityRps = getCurrentRPM();
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

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(FeederState state) {
        setSetpoint(state.rps);
        // Enable dynamic distance-based speed only when spinning forward.
        // Reverse and OFF always use the literal enum value.
        dynamicSpeedActive = (state == FeederState.FORWARD);
    }

    public void setDistanceSupplier(DoubleSupplier supplier) {
        this.distanceToTargetSupplier = supplier;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(FeederState state){
        return this.runOnce(() -> setState(state));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction)
            .beforeStarting(() -> isCharacterizing = true)
            .finallyDo(() -> isCharacterizing = false);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction)
            .beforeStarting(() -> isCharacterizing = true)
            .finallyDo(() -> isCharacterizing = false);
    }

    public enum FeederState{
        FORWARD(22.0),
        REVERSE(-22.0),
        OFF(0);

        double rps;

        private FeederState(double rps) {
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(FeederState.OFF);
        dynamicSpeedActive = false;
    }

}
