package com.spartronics4915.frc2026.subsystems.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.IntakeConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);

    private double currentSetpoint;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("intake").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public IntakeSubsystem() {
        TalonFXConfigurator intakeMotorConfig = motor.getConfigurator();
            intakeMotorConfig.apply(PID_CONFIG);
            intakeMotorConfig.apply(CURRENT_LIMITS_CONFIG);
            intakeMotorConfig.apply(FEEDBACK_CONFIG);
            intakeMotorConfig.apply(MOTOR_OUTPUT_CONFIG);

        setState(IntakeState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Intake On", setStateCommand(IntakeState.INTAKE));
        SmartDashboard.putData("Intake Off", setStateCommand(IntakeState.OFF));
        SmartDashboard.putData("Intake Motor", motor);
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        if (currentSetpoint != 0) {
            velocityVoltage.withEnableFOC(ENABLE_FOC).Velocity = currentSetpoint;
            motor.setControl(velocityVoltage);
        } else {
            motor.setControl(new VoltageOut(0.0));
        }

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPS());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPS() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getAppliedVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
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
        INTAKE(17),
        OUTTAKE(-17),
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
