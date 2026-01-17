import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSubsystem {

    SwerveSubsytemKinematics kinematics;
    private 

    public SwerveSubsystem() {

        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(SwerveConstants.frontLeftWheelDistanceInches), Units.inchesToMeters(SwerveConstants.frontLeftWheelDistanceInches)), //Front Left
            new Translation2d(Units.inchesToMeters(SwerveConstants.frontRightWheelDistanceInches), Units.inchesToMeters(SwerveConstants.frontRightWheelDistanceInches)), //Front Right
            new Translation2d(Units.inchesToMeters(SwerveConstants.backLeftWheelDistanceInches), Units.inchesToMeters(SwerveConstants.backLeftWheelDistanceInches)), //Back Left
            new Translation2d(Units.inchesToMeters(SwerveConstants.backRightWheelDistanceInches), Units.inchesToMeters(SwerveConstants.backRightWheelDistanceInches)), //Back Right
        );
    }
 
    public void drive() {
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(SwerveConstants.testSpeedsX), Units.inchesToMeters(SwerveConstants.testSpeedsY), Units.degreesToRadians(SwerveConstants.testSpeedsDeg));

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
    }
    
}