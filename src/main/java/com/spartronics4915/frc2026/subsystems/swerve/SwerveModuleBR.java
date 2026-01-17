import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SwerveModuleBR {

    private TalonFX driveMotorBR;
    private TalonFX angleMotorBR;
    private CANcoder absoluteEncoderBR;

    public SwerveModuleBR() {

        driveMotorBR = new TalonFX(SwerveConstants.driveMotorBRID);
        angleMotorBR = new TalonFX(SwerveConstants.angleMotorBRID);
        absoluteEncoderBR = new CANcoder(SwerveConstants.absoluteEncoderBRID);

        driveMotorBR.restoreFactoryDefaults();
        angleMotorBR.restoreFactoryDefaults();
        absoluteEncoderBR.getConfigurator().apply(new CANConfiguration());

        CANcoderConfigurator cfg = encoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        driveMotorBR.setInverted(SwerveConstants.driveMotorBRInverted);

        angleMotorBR.setInverted(SwerveConstants.angleMotorBRInverted);
    }
}