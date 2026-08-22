package com.spartronics4915.frc2026.subsystems.mechanisms;

import static com.spartronics4915.frc2026.Constants.IntakeConstants.*;
import static edu.wpi.first.units.Units.Volts;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface {

    // Enable FOC control and Switch to Velocity Voltage

    private LoggedTalonFX leadMotor = new LoggedTalonFX(LEAD_MOTOR_ID, CAN_BUS);

    private double currentSetpoint;

    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);

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
            (Voltage volts) -> leadMotor.setControl(sysIdControl.withOutput(volts.in(Volts))),
            (log) -> {
                log.motor("Intake")
                    .voltage(Volts.of(leadMotor.getTorqueCurrent().getValueAsDouble()))
                    .angularVelocity(leadMotor.getVelocity().getValue())
                    .angularPosition(leadMotor.getPosition().getValue())
                    .angularAcceleration(leadMotor.getAcceleration().getValue());
            },
            this
        )
    );

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("setpoint").publish();

    public IntakeSubsystem() {
        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        ModeSwitchHandler.EnableModeSwitchHandler(this);

        leadMotor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Intake Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Intake Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("Intake Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Intake Dynamic Reverse", sysIdDynamic(Direction.kReverse));

        SmartDashboard.putData("Intake On", setStateCommand(IntakeState.INTAKE));
        SmartDashboard.putData("Intake Off", setStateCommand(IntakeState.OFF));
        SmartDashboard.putData("Intake Motor", leadMotor);
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        if (!isCharacterizing) {
            if (currentSetpoint != 0) {
                velocityTorqueRequest.Velocity = currentSetpoint;
                leadMotor.setControl(velocityTorqueRequest);
            } else {
                leadMotor.setControl(new VoltageOut(0.0));
            }
        }

        appliedOutPublisher.accept(leadMotor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPS());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPS() {
        return leadMotor.getVelocity().getValueAsDouble();
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
