package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc2026.Constants.ShooterConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;
import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.MotorHelpers.CTRE.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private LoggedTalonFX leadMotor;
    private LoggedTalonFX followerMotor;

    private double currentSetpoint;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    private ShooterClamp RPSClamp;
    private double maxRPS;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("shooter").getDoubleTopic("setpoint").publish();

    //#region Main Functionality

    public ShooterSubsystem() {
        leadMotor = new LoggedTalonFX(LEAD_MOTOR_ID, CAN_BUS);
            leadMotor.setNeutralMode(NeutralModeValue.Brake);
        
        followerMotor = new LoggedTalonFX(FOLLOWER_MOTOR_ID, CAN_BUS);
            followerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = leadMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            
        configurator = followerMotor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        followerMotor.setControl(new Follower(LEAD_MOTOR_ID, MotorAlignmentValue.Aligned));

        setClamp(ShooterClamp.UNRESTRICTED);
        
        setSetpointCommand(0);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        leadMotor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Shooter On", setSetpointCommand(55));
        SmartDashboard.putData("Shooter Off", setSetpointCommand(0));
        SmartDashboard.putData("Lead Shooter Motor", leadMotor);
        SmartDashboard.putData("Follower Shooter Motor", followerMotor);
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
                currentSetpoint, 
                -maxRPS, 
                maxRPS
            );

        velocityVoltage.withEnableFOC(ENABLE_FOC).Velocity = currentSetpoint;
        leadMotor.setControl(velocityVoltage);

        appliedOutPublisher.accept(leadMotor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPS());
        setpointPublisher.accept(currentSetpoint);
    }

    public double getCurrentRPS() {
        return leadMotor.getVelocity().getValueAsDouble();
    }

    public ShooterClamp getShooterClamp() {
        return RPSClamp;
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
