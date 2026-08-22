package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static com.spartronics4915.frc2026.Constants.TurretConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.spartronics4915.frc2026.Robot;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.BiConsumer;

public class TurretSubsystem extends SubsystemBase implements ModeSwitchInterface {

    // May need to retune or switch to position + FOC

    private LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    private CANcoder encoder = new CANcoder(ENCODER_ID, CAN_BUS);
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private State targetState = new State();

    private final PositionTorqueCurrentFOC positionTorqueRequest = new PositionTorqueCurrentFOC(0.0);

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
                log.motor("Turret")
                    .voltage(Volts.of(motor.getTorqueCurrent().getValueAsDouble()))
                    .angularPosition(motor.getPosition().getValue())
                    .angularVelocity(motor.getVelocity().getValue())
                    .angularAcceleration(motor.getAcceleration().getValue());
            },
            this
        )
    );
    
    private TurretClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("turret").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("setpoint", Rotation2d.struct).publish();
    private final StructPublisher<Pose3d> pose3dPublisher = NetworkTableInstance.getDefault().getTable("turret").getStructTopic("Pose3d", Pose3d.struct).publish();

    // Vision observer: called during turret's periodic() with accurate timestamp
    private BiConsumer<Rotation2d, Double> visionObserver;
    
    public TurretSubsystem() {

        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
            motorConfigurator.apply(PID_CONFIG);
            motorConfigurator.apply(CURRENT_LIMITS_CONFIG);
            motorConfigurator.apply(FEEDBACK_CONFIG);
            motorConfigurator.apply(MOTOR_OUTPUT_CONFIG); 

        CANcoderConfiguration cancoderConfigurator = new CANcoderConfiguration();
            cancoderConfigurator.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            cancoderConfigurator.MagnetSensor.SensorDirection = ENCODER_SENSOR_DIRECTION;
            cancoderConfigurator.MagnetSensor.MagnetOffset = MAGNET_OFFSET;
            encoder.getConfigurator().apply(cancoderConfigurator);
        
        currentClamp = TurretClamp.UNRESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        setMechanismAngle(Rotation2d.fromDegrees(getEncoderPosition().getDegrees()));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> targetState.position, (setpoint) -> setSetpoint(Rotation2d.fromDegrees(setpoint)));

        SmartDashboard.putData("Turret Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Turret Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("Turret Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Turret Dynamic Reverse", sysIdDynamic(Direction.kReverse));

        SmartDashboard.putData("Turret 0", setSetpointCommand(Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Turret 180", setSetpointCommand(Rotation2d.fromDegrees(180)));
        SmartDashboard.putData("Turret Motor", motor);
    }
 
    @Override
    public void periodic(){
        targetState.position = MathUtil.clamp(
            targetState.position, 
            minAngle.getRotations(), 
            maxAngle.getRotations()
        );

        if (targetState.position <= minAngle.getRotations() || targetState.position >= maxAngle.getRotations()) {
            targetState.velocity = 0;
        }

        positionTorqueRequest
            .withPosition(targetState.position)
            .withVelocity(targetState.velocity);
            
        if (!isCharacterizing) {
            motor.setControl(positionTorqueRequest);
        }

        appliedOutPublisher.accept(motor.getDutyCycle().getValueAsDouble());
        positionPublisher.accept(getPosition());
        setpointPublisher.accept(Rotation2d.fromRotations(targetState.position));
        pose3dPublisher.accept(
            new Pose3d(
                -0.118295, -0.143695, 0.362276, 
                new Rotation3d(0, 0, getPosition().getRadians())
            )
        );

        // Notify vision of turret angle with accurate FPGA timestamp
        if (visionObserver != null) {
            visionObserver.accept(getPosition(), Timer.getFPGATimestamp());
        }
    }

    public Rotation2d getPosition() {
        if (Robot.isSimulation()) {
            return Rotation2d.fromRotations(targetState.position);
        }
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public Rotation2d getEncoderPosition() {
        double position = encoder.getAbsolutePosition().getValue().in(Rotations) * ENCODER_MECHANISM_RATIO;
        return Rotation2d.fromRotations(position);
    }

    public Rotation2d getCurrentSetpoint() {
        return Rotation2d.fromRotations(targetState.position);
    }

    public TurretClamp getClamp() {
        return currentClamp;
    }
    
    public void setSetpoint(Rotation2d setpoint){
        targetState.position = setpoint.getRotations();
        targetState.velocity = 0.0;
    }

    public void setComplexSetpoint(Rotation2d setPoint, AngularVelocity velocity){
        targetState.position = setPoint.getRotations();
        targetState.velocity = velocity.in(RotationsPerSecond);
    }

    public void setClamp(TurretClamp clamp){
        currentClamp = clamp;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        resetMechanism(angle);
    }

    public void resetMechanism(){
        resetMechanism(getPosition());
    }

    public void resetMechanism(Rotation2d angle){
        setSetpoint(angle);
    }

    /**
     * Registers a vision observer to receive turret angle updates with accurate timestamps.
     * Called by VisionSubsystem during initialization.
     * @param observer BiConsumer that takes (turretAngle, fpgaTimestamp)
     */
    public void setVisionObserver(BiConsumer<Rotation2d, Double> observer) {
        this.visionObserver = observer;
    }

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setClampCommand(TurretClamp newClamp) {
        return this.runOnce(() -> setClamp(newClamp));
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

    //#endregion

    public enum TurretClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(230));

        public Rotation2d minAngle;
        public Rotation2d maxAngle;

        private TurretClamp(Rotation2d minAngle, Rotation2d maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}
