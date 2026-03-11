package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ShooterConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface {

    // --- Hardware ---
    private final LoggedTalonFX leadMotor;
    private final LoggedTalonFX followerMotor;

    // --- Control requests ---
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    // --- Debouncers ---
    private final Debouncer torqueCurrentDebouncer = new Debouncer(TORQUE_CURRENT_DEBOUNCE_SEC, DebounceType.kFalling);
    private final Debouncer atGoalDebouncer = new Debouncer(AT_GOAL_DEBOUNCE_SEC, DebounceType.kFalling);

    // --- State ---
    private double currentSetpoint = 0.0;
    private double maxRPS;
    private ShooterClamp RPSClamp;
    private boolean lastTorqueCurrentControl = false;
    private boolean atGoal = false;
    private long launchCount = 0;

    // --- Telemetry ---
    private final DoublePublisher appliedOutPublisher =
        NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher =
        NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher =
        NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("setpoint").publish();
    private final DoublePublisher launchCountPublisher =
        NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("launch count").publish();
    private final DoublePublisher modePublisher =
        NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("bang bang mode").publish();

    // -------------------------------------------------------------------------

    public ShooterSubsystem() {
        leadMotor = new LoggedTalonFX(LEAD_MOTOR_ID, CAN_BUS);
        followerMotor = new LoggedTalonFX(FOLLOWER_MOTOR_ID, CAN_BUS);

        leadMotor.setNeutralMode(NeutralModeValue.Brake);
        followerMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);

        configurator = followerMotor.getConfigurator();
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        followerMotor.setControl(new Follower(LEAD_MOTOR_ID, MotorAlignmentValue.Aligned));

        setClamp(ShooterClamp.UNRESTRICTED);
        setSetpoint(0);
        ModeSwitchHandler.EnableModeSwitchHandler(this);
        leadMotor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Shooter On",  setSetpointCommand(55));
        SmartDashboard.putData("Shooter Off", setSetpointCommand(0));
        SmartDashboard.putData("Lead Shooter Motor",     leadMotor);
        SmartDashboard.putData("Follower Shooter Motor", followerMotor);
    }

    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(currentSetpoint, -maxRPS, maxRPS);
        final double velocity = getCurrentRPS();

        if (currentSetpoint != 0) {
            final double error = currentSetpoint - velocity;
            final boolean inTolerance = Math.abs(error) <= BANG_BANG_TOLERANCE_RPS;
            final boolean useTorqueCurrent = torqueCurrentDebouncer.calculate(inTolerance);

            atGoal = atGoalDebouncer.calculate(inTolerance);

            if (!useTorqueCurrent && lastTorqueCurrentControl) launchCount++;
            lastTorqueCurrentControl = useTorqueCurrent;

            if (useTorqueCurrent) {
                // if near setpoint use direct current control for a gentle, precise hold.
                // Apply holding current when slightly under, coast when at/above.
                leadMotor.setControl(velocity < currentSetpoint
                    ? torqueCurrentFOC.withOutput(HOLDING_TORQUE_CURRENT_AMPS)
                    : voltageOut.withOutput(0.0));
                modePublisher.accept(1.0); // 1 = torque-current mode

            } else {
                // if far from setpoint ramp duty cycle down linearly as we approach the
                // tolerance band, rather than cutting hard from 100% to 0%. This prevents
                // the flywheel's inertia from carrying it past the setpoint on every spin-up
                if (velocity < currentSetpoint) {
                    double output = (error > RAMP_DOWN_WINDOW_RPS) ? 1.0 : (error / RAMP_DOWN_WINDOW_RPS);
                    leadMotor.setControl(dutyCycleOut.withOutput(output).withEnableFOC(ENABLE_FOC));
                } else {
                    leadMotor.setControl(voltageOut.withOutput(0.0));
                }
                modePublisher.accept(0.0); // duty-cycle mode
            }

        } else {
            // if setpoint is zero, coast and clear goal state.
            leadMotor.setControl(voltageOut.withOutput(0.0));
            atGoal = false;
            lastTorqueCurrentControl = false;
            modePublisher.accept(-1.0); // stopped
        }

        // Telemetry
        appliedOutPublisher.accept(leadMotor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(velocity);
        setpointPublisher.accept(currentSetpoint);
        launchCountPublisher.accept(launchCount);
    }

    // Getters -----------------------------------------------------

    public double getCurrentRPS() { 
        return leadMotor.getVelocity().getValueAsDouble(); 
    }

    public double getCurrentSetpoint() { 
        return currentSetpoint; 
    }

    public ShooterClamp getShooterClamp() { 
        return RPSClamp; 
    }

    public boolean atGoal() { 
        return atGoal; 
    }

    public long getLaunchCount() { 
        return launchCount; 
    }

    // Setters -----------------------------------------------------

    public void setSetpoint(double setpoint) { 
        currentSetpoint = setpoint; 
    }

    public void setClamp(ShooterClamp clamp) { 
        RPSClamp = clamp; maxRPS = clamp.maxRPS; 
    }

    // Commands -----------------------------------------------------

    public Command setSetpointCommand(double setpoint) { 
        return runOnce(() -> setSetpoint(setpoint)); 
    }

    public Command setClampCommand(ShooterClamp clamp) { 
        return runOnce(() -> setClamp(clamp)); 
    }

    // -------------------------------------------------------------------------

    public enum ShooterClamp {
        RESTRICTED(0), UNRESTRICTED(100);
        final double maxRPS;
        ShooterClamp(double maxRPS) { this.maxRPS = maxRPS; }
    }

    @Override
    public void onModeSwitch() { 
        setSetpoint(0); 
    }

}