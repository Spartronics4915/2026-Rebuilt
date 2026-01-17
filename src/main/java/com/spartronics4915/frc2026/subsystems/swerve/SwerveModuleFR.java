import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SwerveModuleFR {

    private TalonFX driveMotorFR;
    private TalonFX angleMotorFR;
    private CANcoder absoluteEncoderFR;

    public SwerveModuleFL() {

        driveMotorFR = new TalonFX(SwerveConstants.driveMotorFRID);
        angleMotorFR = new TalonFX(SwerveConstants.angleMotorFRID);
        absoluteEncoderFR = new CANcoder(SwerveConstants.absoluteEncoderFRID);

        driveMotorFR.restoreFactoryDefaults();
        angleMotorFR.restoreFactoryDefaults();
        absoluteEncoderFR.getConfigurator().apply(new CANConfiguration());

        CANcoderConfigurator cfg = encoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        driveMotorFR.setInverted(SwerveConstants.driveMotorFRInverted);

        angleMotorFR.setInverted(SwerveConstants.angleMotorFRInverted);
    }
}