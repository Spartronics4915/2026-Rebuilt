package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private TalonFX leadMotor;
    private TalonFX followerMotor;

    private SlewRateLimiter RPSLimiter = new SlewRateLimiter(MAX_ACCELERATION);

    private double currentSetpoint;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public ShooterSubsystem() {
        this.currentSetpoint = 0.0;

        leadMotor = new TalonFX(LEAD_MOTOR_ID);
        leadMotor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);

        followerMotor = new TalonFX(FOLLOWER_MOTOR_ID);
            
        configurator = followerMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);

        followerMotor.setControl(new Follower(FOLLOWER_MOTOR_ID, MotorAlignmentValue.Aligned));
        
        leadMotor.set(0);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("Shooter On", setStateCommand(ShooterState.ON));
        SmartDashboard.putData("Shooter Off", setStateCommand(ShooterState.OFF));
    }

    @Override
    public void periodic() {
        double limitedSetpoint;
        if (currentSetpoint != 0) {
            limitedSetpoint = RPSLimiter.calculate(currentSetpoint);
            VelocityVoltage request = new VelocityVoltage(limitedSetpoint);
            leadMotor.setControl(request);
        } else {
            limitedSetpoint = 0;
            RPSLimiter.reset(0);
            VoltageOut request = new VoltageOut(0.0);
            leadMotor.setControl(request);
        }

        appliedOutPublisher.accept(leadMotor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPS());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPS() {
        return leadMotor.getVelocity().getValueAsDouble();
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(ShooterState state) {
        setSetpoint(state.rps);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(ShooterState state){
        return setSetpointCommand(state.rps);
    }

    public enum ShooterState{
        ON(100),
        OFF(0);

        double rps;

        private ShooterState(double rps) {
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(ShooterState.OFF);
    }

}
