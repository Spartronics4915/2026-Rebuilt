package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ShooterConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private TalonFX leadMotor;
    private TalonFX followerMotor;

    private SlewRateLimiter RPSLimiter = new SlewRateLimiter(MAX_ACCELERATION);

    private double currentSetpoint;

    private ShooterClamp RPSClamp;
    private double maxRPS;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("rps").publish();
    private final DoublePublisher desiredStatePublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("desired State").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public ShooterSubsystem() {
        leadMotor = new TalonFX(LEAD_MOTOR_ID, CAN_BUS);
            leadMotor.setNeutralMode(NeutralModeValue.Brake);
        
        followerMotor = new TalonFX(FOLLOWER_MOTOR_ID, CAN_BUS);
            followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(OUTPUT_CONFIG);
            
        configurator = followerMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(OUTPUT_CONFIG);

        followerMotor.setControl(new Follower(LEAD_MOTOR_ID, MotorAlignmentValue.Aligned));

        setClamp(ShooterClamp.UNRESTRICTED);
        
        setSetpointCommand(0);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("Shooter On", setSetpointCommand(52));
        SmartDashboard.putData("Shooter Off", setSetpointCommand(0));
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
                currentSetpoint, 
                -maxRPS, 
                maxRPS
            );

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
        desiredStatePublisher.accept(limitedSetpoint);
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

    public void setClamp(ShooterClamp clamp) {
        RPSClamp = clamp;
        maxRPS = RPSClamp.maxRPS;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint) {
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setClampCommand(ShooterClamp clamp) {
        return this.runOnce(() -> setClamp(clamp));
    }

    public enum ShooterClamp{
        RESTRICTED(0),
        UNRESTRICTED(100);

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
