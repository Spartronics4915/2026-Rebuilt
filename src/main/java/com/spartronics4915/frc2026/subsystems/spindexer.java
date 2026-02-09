package com.spartronics4915.frc2026.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class spindexer extends SubsystemBase{
    
    TalonFX spindexerMotor = new TalonFX(Constants.SpindexerConstants.SPINDEXER_MOTOR_ID);
    TalonFXConfigurator spindexerConfigurator = spindexerMotor.getConfigurator();

    State spindexerState = new State(Constants.SpindexerConstants.SPINDEXER_POSITION, Constants.SpindexerConstants.SPINDEXER_VELOCITY);

    double setPoint = (0.0);

    TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
        new Constraints(Constants.SpindexerConstants.SPINDEXER_MAX_VELOCITY, Constants.SpindexerConstants.SPINDEXER_MAX_ACCELERATION)
    );

    DoublePublisher spindexerVoltage = NetworkTableInstance.getDefault().getDoubleTopic("Spindexer Voltage").publish(null);
    DoublePublisher spindexerRPM = NetworkTableInstance.getDefault().getDoubleTopic("Spindexer RPM").publish(null);

    private void initialize(){
        SmartDashboard.putBoolean("on", true);
        spindexerConfigurator.apply( new SlotConfigs()
            .withKP(Constants.SpindexerConstants.SPINDEXER_P)
            .withKI(Constants.SpindexerConstants.SPINDEXER_I)
            .withKD(Constants.SpindexerConstants.SPINDEXER_D)
        );
        spindexerConfigurator.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.SpindexerConstants.SPINDEXER_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.SpindexerConstants.SPINDEXER_LOWER_CURRENT_LIMIT)
            .withSupplyCurrentLowerTime(Constants.SpindexerConstants.SPINDEXER_LOWER_CURRENT_TIME)
            .withSupplyCurrentLimitEnable(Constants.SpindexerConstants.SPINDEXER_CURRENT_LIMIT_ENABLE)
        );
        spindexerConfigurator.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.SpindexerConstants.SPINDEXER_SENSOR_TO_MECHANISM_RATIO)
        );

    }

    public void spindexer() {
        initialize();
    }

    public double getVolt(){
        return spindexerMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getRPM(){
        return spindexerMotor.getAcceleration().getValueAsDouble();
    }

    public void spindexerOn(){
        setPoint = Constants.SpindexerConstants.SPINDEXER_MOTOR_SPEED;
    }

    public void spindexerOff(){
        setPoint = 0;
    }

    @Override
    public void periodic() {
        setPoint = MathUtil.clamp(
            setPoint, 
            Constants.SpindexerConstants.SPINDEXER_MAX_VELOCITY, 
            Constants.SpindexerConstants.SPINDEXER_MIN_VELOCITY
        );

        VelocityVoltage VVrequest = new VelocityVoltage(setPoint);

        spindexerMotor.setControl(VVrequest);
        spindexerVoltage.accept(this.getVolt());
        spindexerRPM.accept(this.getRPM());
        if (SmartDashboard.getBoolean("on", false)){
            spindexerOn();
        } else {
            spindexerOff();
        }
    }

}
