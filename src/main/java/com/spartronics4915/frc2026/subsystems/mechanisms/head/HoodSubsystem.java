package com.spartronics4915.frc2026.subsystems.mechanisms.head;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import static com.spartronics4915.frc2026.Constants.HoodConstants.*;
import static com.spartronics4915.frc2026.Constants.GeneralConstants.CAN_BUS;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.spartronics4915.frc2026.util.general.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.general.ModeSwitchHandler.ModeSwitchInterface;
import com.spartronics4915.frc2026.util.mechanism.TimeVarianceAuthority;
import com.spartronics4915.frc2026.util.mechanism.MotorHelpers.CTRE.LoggedTalonFX;

public class HoodSubsystem extends SubsystemBase implements ModeSwitchInterface {

    LoggedTalonFX motor = new LoggedTalonFX(MOTOR_ID, CAN_BUS);
    
    TimeVarianceAuthority dtCalc = new TimeVarianceAuthority();

    private State targetState = new State();
    
    private TurretSubsystem turret;

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
                log.motor("Hood")
                    .voltage(Volts.of(motor.getTorqueCurrent().getValueAsDouble()))
                    .angularPosition(motor.getPosition().getValue())
                    .angularVelocity(motor.getVelocity().getValue())
                    .angularAcceleration(motor.getAcceleration().getValue());
            },
            this
        )
    );

    private HoodClamp currentClamp;
    private Rotation2d minAngle;
    private Rotation2d maxAngle;

    private final DoublePublisher appliedOutPublisher = NetworkTableInstance.getDefault().getTable("hood").getDoubleTopic("applied out").publish();
    private final StructPublisher<Rotation2d> positionPublisher = NetworkTableInstance.getDefault().getTable("hood").getStructTopic("position", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> setpointPublisher = NetworkTableInstance.getDefault().getTable("hood").getStructTopic("setpoint", Rotation2d.struct).publish();
    private final StructPublisher<Pose3d> pose3dPublisher = NetworkTableInstance.getDefault().getTable("hood").getStructTopic("Pose3d", Pose3d.struct).publish();
    
    public HoodSubsystem() {
        TalonFXConfigurator motorConfig = motor.getConfigurator();
            motorConfig.apply(PID_CONFIG);
            motorConfig.apply(CURRENT_LIMITS_CONFIG);
            motorConfig.apply(FEEDBACK_CONFIG);
            motorConfig.apply(MOTOR_OUTPUT_CONFIG);

        currentClamp = HoodClamp.UNRESTRICTED;
            minAngle = currentClamp.minAngle;
            maxAngle = currentClamp.maxAngle;

        setMechanismAngle(Rotation2d.fromDegrees(0));
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        motor.addSetpoint(() -> targetState.position, (setpoint) -> setSetpoint(Rotation2d.fromDegrees(setpoint)));

        SmartDashboard.putData("Hood Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Hood Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
        SmartDashboard.putData("Hood Dynamic Forward", sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Hood Dynamic Reverse", sysIdDynamic(Direction.kReverse));

        SmartDashboard.putData("Hood Up", setSetpointCommand(Rotation2d.fromDegrees(19)));
        SmartDashboard.putData("Hood Down", setSetpointCommand(Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("Hood Motor", motor);
    }

    public void setTurretSubsystem(TurretSubsystem turretSubsystem) {
        this.turret = turretSubsystem;
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
        
        // Publish 3D pose with turret rotation applied
        double turretAngle = (turret != null) ? turret.getPosition().getRadians() : 0.0;
        Rotation3d hoodRotation = new Rotation3d(-getPosition().getRadians(), 0, turretAngle);
        pose3dPublisher.accept(
            new Pose3d(-0.1181, -0.0972-0.0453, 0.4953 + getPosition().getTan() * 0.0453, hoodRotation)
        );
    }

    public Rotation2d getPosition() {
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public Rotation2d getCurrentSetpoint() {
        return Rotation2d.fromRotations(targetState.position);
    }

    public void setSetpoint(Rotation2d setpoint){
        targetState.position = setpoint.getRotations();
        targetState.velocity = 0.0;
    }

    public void setComplexSetpoint(Rotation2d setPoint, AngularVelocity velocity){
        targetState.position = setPoint.getRotations();
        targetState.velocity = velocity.in(RotationsPerSecond);
    }

    public void setClamp(HoodClamp clamp){
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

    //#endregion

    //#region Commands

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command setClampCommand(HoodClamp newClamp) {
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
 
    public enum HoodClamp {
        RESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        UNRESTRICTED(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(40));

        Rotation2d minAngle;
        Rotation2d maxAngle;

        private HoodClamp(Rotation2d minAngle, Rotation2d maxAngle) {
            this.minAngle = minAngle;
            this.maxAngle = maxAngle;
        }
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}
    // This is so awesome!

    //  H   H  OOO   OOO  DDDD      CCC   OOO  DDDD  EEEEE   //
    //  H   H O   O O   O D   D    C   C O   O D   D E       //
    //  H   H O   O O   O D   D    C     O   O D   D E       //
    //  HHHHH O   O O   O D   D    C     O   O D   D EEEE    //
    //  H   H O   O O   O D   D    C     O   O D   D E       //
    //  H   H O   O O   O D   D    C   C O   O D   D E       //
    //  H   H  OOO   OOO  DDDD      CCC   OOO  DDDD  EEEEE   //
