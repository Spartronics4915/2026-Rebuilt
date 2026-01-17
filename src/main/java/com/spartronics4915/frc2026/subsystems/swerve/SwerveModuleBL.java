import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class SwerveModuleBL {

    private TalonFX driveMotorBL;
    private TalonFX angleMotorBL;H
    private CANcoder absoluteEncoderBL;

    public SwerveModuleBL() {

        driveMotorBL = new TalonFX(SwerveConstants.driveMotorBLID);
        angleMotorBL = new TalonFX(SwerveConstants.angleMotorBLID);
        absoluteEncoderBL = new CANcoder(SwerveConstants.absoluteEncoderBLID);

        driveMotorBL.restoreFactoryDefaults();
        angleMotorBL.restoreFactoryDefaults();
        absoluteEncoderBL.getConfigurator().apply(new CANConfiguration());
    }
}