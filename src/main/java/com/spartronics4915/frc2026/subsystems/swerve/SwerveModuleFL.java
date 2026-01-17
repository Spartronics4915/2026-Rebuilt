import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SwerveModuleFL {

    private TalonFX driveMotorFL;
    private TalonFX angleMotorFL;
    private CANcoder absoluteEncoderFL;

    public SwerveModuleFL() {

        driveMotorFL = new TalonFX(SwerveConstants.driveMotorFLID);
        angleMotorFL = new TalonFX(SwerveConstants.angleMotorFLID);
        absoluteEncoderFL = new CANcoder(SwerveConstants.absoluteEncoderFLID);

        driveMotorFL.restoreFactoryDefaults();
        angleMotorFL.restoreFactoryDefaults();
        absoluteEncoderFL.getConfigurator().apply(new CANConfiguration());
    }
}