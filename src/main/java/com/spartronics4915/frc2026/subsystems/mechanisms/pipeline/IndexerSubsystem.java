package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import static edu.wpi.first.units.Units.Volts;

import static com.spartronics4915.frc2026.Constants.IndexerConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class IndexerSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    // Enable FOC control and Switch to Velocity Voltage

    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);

    private double currentSetpoint;
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0.0);
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(50);
    
    private double indexerAngle = 0.0; // Tracks cumulative rotation angle in radians

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
                log.motor("Indexer")
                    .voltage(Volts.of(motor.getTorqueCurrent().getValueAsDouble()))
                    .angularVelocity(motor.getVelocity().getValue())
                    .angularPosition(motor.getPosition().getValue())
                    .angularAcceleration(motor.getAcceleration().getValue());
            },
            this
        )
    );

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("applied out").publish();
    private final DoublePublisher rpsPublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("rps").publish();
    private final DoublePublisher setpointPublisher = NetworkTableInstance.getDefault().getTable("indexer").getDoubleTopic("setpoint").publish();
    private final StructPublisher<Pose3d> pose3dPublisher = NetworkTableInstance.getDefault().getTable("indexer").getStructTopic("Pose3d", Pose3d.struct).publish();

    public IndexerSubsystem() {
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PID_CONFIG);
            configurator.apply(CURRENT_LIMITS_CONFIG);
            configurator.apply(FEEDBACK_CONFIG);
            configurator.apply(MOTOR_OUTPUT_CONFIG);

        setState(IndexerState.OFF);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> currentSetpoint, this::setSetpoint);

        SmartDashboard.putData("Indexer Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Indexer Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("Indexer Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Indexer Dynamic Reverse", sysIdDynamic(Direction.kReverse));
        
        SmartDashboard.putData("Indexer On", setStateCommand(IndexerState.FORWARD));
        SmartDashboard.putData("Indexer Off", setStateCommand(IndexerState.OFF));
        SmartDashboard.putData("Indexer Motor", motor);
    }

    @Override
    public void periodic() {
        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            -MAX_RPS,
            MAX_RPS
        );

        double limitedSetpoint = slewRateLimiter.calculate(currentSetpoint);

        if (!isCharacterizing) {
            if (limitedSetpoint != 0) {
                velocityTorqueRequest.Velocity = limitedSetpoint;
                motor.setControl(velocityTorqueRequest);
            } else {
                motor.setControl(new VoltageOut(0.0));
            }
        }
        
        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        rpsPublisher.accept(getCurrentRPS());
        setpointPublisher.accept(currentSetpoint);
        
        // Publish 3D pose with rotation around Z-axis (perpendicular to intake flow)

        indexerAngle += getCurrentRPS() * 2 * Math.PI * 0.02;

        pose3dPublisher.accept(
            new Pose3d(
                0.0472, -0.0002, 0.0619, 
                new Rotation3d(0, 0, indexerAngle)
            )
        );
    }

    public double getCurrentRPS() {
        return Robot.isReal() ? motor.getVelocity().getValueAsDouble() : getCurrentSetpoint();
    }

    public double getCurrentSetpoint() {
        return currentSetpoint;
    }

    public void setSetpoint(double setpoint){
        currentSetpoint = setpoint;
    }

    public void setState(IndexerState state) {
        setSetpoint(state.rps);
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(double setpoint){
        return this.runOnce(() -> setSetpoint(setpoint));
    }

    public Command setStateCommand(IndexerState state){
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

    public enum IndexerState {
        FORWARD(22.0),
        REVERSE(-22.0),
        OFF(0.0);

        public double rps;
        private IndexerState(double rps) { 
            this.rps = rps;
        }
    }

    @Override
    public void onModeSwitch() {
        setState(IndexerState.OFF);
    }

}

