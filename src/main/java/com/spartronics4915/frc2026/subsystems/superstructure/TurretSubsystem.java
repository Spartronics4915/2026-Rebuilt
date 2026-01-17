package com.spartronics4915.frc2026.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.spartronics4915.frc2026.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase{
    DoublePublisher turretSetpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Turret Current Setpoint").publish();
    DoublePublisher turretAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Turret Angle").publish();
    DoublePublisher turretOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Turret Output").publish();
    DoublePublisher turretRequestedPosPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Turret Requested Position").publish();


    TalonFX turretMotor = new TalonFX(Constants.TurretConstants.TURRET_MOTOR_ID);
    TalonFXConfigurator turretConfigurator = turretMotor.getConfigurator();

    public TurretSubsystem(){
      applyMotorConfigs(turretConfigurator);  

    }
    
    private void applyMotorConfigs(TalonFXConfigurator config){
        config.apply(new SlotConfigs()
            .withKP(Constants.TurretConstants.TURRET_P)
            .withKI(Constants.TurretConstants.TURRET_I)
            .withKD(Constants.TurretConstants.TURRET_D)
        );
        config.apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.TurretConstants.CURRENT_LIMIT_ENABLED)
            .withSupplyCurrentLimit(Constants.TurretConstants.SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(Constants.TurretConstants.CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(Constants.TurretConstants.CUREENT_LOWER_TIME)
        );
        config.apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.TurretConstants.SENSOR_TO_MECHANISM_RATIO)
        );

    }

    TrapezoidProfile trapProfile = new TrapezoidProfile(new Constraints(Constants.TurretConstants.MAX_VELOCITY,Constants.TurretConstants.MAX_ACCELERATION));
    double position = this.getAngle().getRotations();
    State currentState = new State(position,0);
    double currentSetPoint = position;
   
    @Override
    public void periodic(){
        currentSetPoint = MathUtil.clamp(
            currentSetPoint,
            Constants.TurretConstants.MIN_ROTATION,
            Constants.TurretConstants.MAX_ROTATION
        );
        currentState = trapProfile.calculate(
            Constants.TurretConstants.DELTA_TIME, 
            currentState, 
            new State(currentSetPoint,0)
        );
        PositionVoltage request = new PositionVoltage(currentState.position);

        turretMotor.setControl(request);
        publishData();
    }

    private void publishData(){
        turretSetpointPublisher.accept(currentSetPoint);
        turretAnglePublisher.accept(turretMotor.getPosition().getValue().in(Degrees));
        turretOutputPublisher.accept(turretMotor.getClosedLoopOutput().getValueAsDouble());
        turretRequestedPosPublisher.accept(currentState.position);
    }

   /**
    * Sets the turrets rotation value
    * @param input the desiered rotation value, in degrees
    */
    public Command setTurret(Rotation2d input){
        return this.runOnce(()->currentSetPoint = input.getRotations());
    }
    /**
     * increments the turret's angle 
     * @param input the amount you wish to modify the turret's angle by, in degrees 
     */
    public Command incrementTurret(Rotation2d input){
        return this.runOnce(()->currentSetPoint += input.getRotations());
    }
    /**
     * Gets the angle of the turret in degrees
     * @return the angle of the turret in degrees
     */
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(turretMotor.getPosition().getValue().in(Degrees));
    }

}