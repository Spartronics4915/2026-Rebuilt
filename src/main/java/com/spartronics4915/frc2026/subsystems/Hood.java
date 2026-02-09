package com.spartronics4915.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.spartronics4915.frc2026.Constants;

public class Hood extends SubsystemBase {
    
    DoublePublisher hoodAnglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood Angle").publish();
    DoublePublisher hoodVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood Voltage").publish();
    TrapezoidProfile trapProfile = new TrapezoidProfile(
	    new Constraints(Constants.HoodConstants.HOOD_MAX_VELOCITY, Constants.HoodConstants.HOOD_MAX_ACCELERATION)
    );

    State currentState = new State(0, 0);
    
    TalonFX robotsonTheThird = new TalonFX(Constants.HoodConstants.HOOD_MOTOR_ID);
    public Hood() {
        TalonFXConfigurator motorConfig = robotsonTheThird.getConfigurator();
        motorConfig.apply(new SlotConfigs()
                .withKP(Constants.HoodConstants.P)
                .withKI(Constants.HoodConstants.I)
                .withKD(Constants.HoodConstants.D));
        motorConfig.apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(Constants.HoodConstants.HOOD_CURRENT_LIMIT_ENABLE)
                .withSupplyCurrentLimit(Constants.HoodConstants.HOOD_CURRENT_LIMIT)
                .withSupplyCurrentLowerLimit(Constants.HoodConstants.HOOD_LOWER_LIMIT)
                .withSupplyCurrentLowerTime(Constants.HoodConstants.HOOD_LOWER_TIME));
        motorConfig.apply(new FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.HoodConstants.HOOD_SENSOR_MECHANISM_RATIO));
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.apply(motorOutputConfigs);

    SmartDashboard.putNumber("Robotson", 0);

    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(robotsonTheThird.getPosition()
                .getValue().in(Degrees));
    }

    public AngularVelocity getSpeed() {
        return RPM.of(robotsonTheThird.getVelocity().getValue().in(RPM));
    }

    private Rotation2d rawToAngle(double rotation) {
        Rotation2d angle = Rotation2d.fromRotations(rotation);
        return angle;
    }
    
    private double angleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    public void setSetpoint(double newSetpoint){
        double currentSetpoint = newSetpoint;
    }

    @Override
    public void periodic(){
        double currentSetpoint = SmartDashboard.getNumber("Robotson", 0);
    
        currentSetpoint = MathUtil.clamp(
            currentSetpoint,
            Constants.HoodConstants.HOOD_MIN,
            Constants.HoodConstants.HOOD_MAX
        );

        currentState = trapProfile.calculate(
            Constants.HoodConstants.HOOD_DT, 
            currentState, 
            new State(currentSetpoint, 0)
        );

        PositionVoltage request = new PositionVoltage(
            currentState.position
        );

        robotsonTheThird.setControl(request);

        hoodAnglePublisher.accept(robotsonTheThird.getPosition().getValueAsDouble());
        hoodVoltagePublisher.accept(robotsonTheThird.getMotorVoltage().getValueAsDouble());

       // robotsonTheThird.setControl(request);

    }

    public Command setSetpointCommand(double newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }
 
    @SuppressWarnings("unused")
    private void setVoltage(double volts){
        robotsonTheThird.setVoltage(volts);
    }

}










    //   H      H   OOOOOO    OOOOOO   DDDDDD          CCCCCC    OOOOOO   DDDDDD    EEEEEEEE   //
    //   H      H  O      O  O      O  D     D        C      C  O      O  D     D   E          //
    //   H      H  O      O  O      O  D      D       C         O      O  D      D  E          //
    //   HHHHHHHH  O      O  O      O  D      D       C         O      O  D      D  EEEEEEE    //
    //   H      H  O      O  O      O  D      D       C         O      O  D      D  E          //
    //   H      H  O      O  O      O  D     D        C      C  O      O  D     D   E          //
    //   H      H   OOOOOO    OOOOOO   DDDDDD          CCCCCC    OOOOOO   DDDDDD    EEEEEEEE   //