package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import static edu.wpi.first.units.Units.Volts;

import static com.spartronics4915.frc2026.Constants.FeederConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class FeederSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    
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

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("feeder").getDoubleTopic("setpoint").publish();

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
        SmartDashboard.putData("Feeder Motor", motor);
    }

    private LaserCan laserCan = new LaserCan(LASER_ID);

    private double getCanOutput(){
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if(measurement == null){
            return 99;
        } else{
            return measurement.distance_mm;
        }
    }

    private boolean isBallDetected(){
        return getCanOutput() <= DETECTION_DISTANCE;
    };

    private double lastTime = 0.0;    
    private  double ballDetectedPercent = 0.0;
    
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
                motor.setControl(velocityTorqueRequest);
            } else {
                motor.setControl(new VoltageOut(0.0));
            }
        }

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPM());
        setpointPublisher.accept(currentSetpoint);

        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        ballDetectedPercent *= (1-deltaTime);
        if(isBallDetected()){
            ballDetectedPercent += deltaTime;
        }
    }

    public double getCurrentRPM() {
        return motor.getVelocity().getValueAsDouble();
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(FeederState state) {
        setSetpoint(state.rps);
    }

    public double getBallPercent(){
        return ballDetectedPercent;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(FeederState state){
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
    }

}