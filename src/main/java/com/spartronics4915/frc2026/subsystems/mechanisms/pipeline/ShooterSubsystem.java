package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import static com.spartronics4915.frc2026.Constants.ShooterConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.MotorHelpers.CTRE.LoggedTalonFX;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private static final Scope LOG = Telemetry.scope("Mechanisms/Shooter");

    private LoggedTalonFX leadMotor;
    private LoggedTalonFX followerMotor;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Double> dutyCycleSignal;
    private final BaseStatusSignal[] telemetrySignals;

    private double currentSetpoint;
    private long sampleTimestampUs;
    private double appliedDutyCycle;
    private double velocityRps;
    private double workingSetpointRps;
    private double profileSetpointRps;

    private SlewRateLimiter rpsProfile = new SlewRateLimiter(9999, maxShooterDecel, 0);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final Follower followerRequest = new Follower(LEAD_MOTOR_ID, MotorAlignmentValue.Aligned);
    private final VoltageOut stopRequest = new VoltageOut(0.0);

    private ShooterClamp RPSClamp;
    private double maxRPS;

    public ShooterSubsystem() {
        leadMotor = new LoggedTalonFX(LEAD_MOTOR_ID, CAN_BUS);
        followerMotor = new LoggedTalonFX(FOLLOWER_MOTOR_ID, CAN_BUS);

        velocitySignal = leadMotor.getVelocity(false);
        dutyCycleSignal = leadMotor.getDutyCycle(false);
        telemetrySignals = leadMotor.createTelemetrySignalGroup(velocitySignal, dutyCycleSignal);

        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        configurator = followerMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);
        followerMotor.setControl(followerRequest);

        setClamp(ShooterClamp.UNRESTRICTED);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        leadMotor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Shooter Motor", leadMotor);

        SmartDashboard.putData("Shooter On", setSetpointCommand(55));
        SmartDashboard.putData("Shooter Off", setSetpointCommand(0));
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(telemetrySignals);

        currentSetpoint = MathUtil.clamp(currentSetpoint, -maxRPS, maxRPS);

        double workingSetpoint = currentSetpoint;
        if (currentSetpoint == 0 /* && !Robot.isPureTeleop */) {
            workingSetpoint = IDLE_SHOOTER_RPS;
        }

        double limitedSetpoint = rpsProfile.calculate(workingSetpoint);

        if (limitedSetpoint != 0) {
            leadMotor.setControl(velocityVoltage.withVelocity(limitedSetpoint));
        } else {
            leadMotor.setControl(stopRequest);
        }

        appliedDutyCycle = dutyCycleSignal.getValueAsDouble();
        velocityRps = Robot.isSimulation() ? currentSetpoint : velocitySignal.getValueAsDouble();
        workingSetpointRps = workingSetpoint;
        profileSetpointRps = limitedSetpoint;
        sampleTimestampUs = RobotController.getFPGATime();
        outputTelemetry();
    }

    private void outputTelemetry() {
        LOG.critical.log("SampleTimestampUs", sampleTimestampUs);
        LOG.critical.log("VelocityRps", velocityRps);
        LOG.critical.log("SetpointRps", workingSetpointRps);
        LOG.critical.log("ProfileSetpointRps", profileSetpointRps);
        LOG.info.log("AppliedDutyCycle", appliedDutyCycle);
    }

    public double getCurrentRPS() {
        if (Robot.isSimulation()) {
            return currentSetpoint;
        }
        return velocityRps;
    }

    public ShooterClamp getShooterClamp() {
        return RPSClamp;
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setSetpoint(double setpoint) {
        currentSetpoint = setpoint;
    }

    public void setClamp(ShooterClamp clamp) {
        RPSClamp = clamp;
        maxRPS = RPSClamp.maxRPS;
    }

    public Command setSetpointCommand(double setpoint) {
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setClampCommand(ShooterClamp clamp) {
        return this.runOnce(() -> setClamp(clamp));
    }

    public enum ShooterClamp {
        RESTRICTED(35), UNRESTRICTED(110);

        double maxRPS;

        private ShooterClamp(double maxRPS) {
            this.maxRPS = maxRPS;
        }
    }

    @Override
    public void onModeSwitch() {
        setSetpoint(0);
    }

}
