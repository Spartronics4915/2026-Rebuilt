import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSubsystem {

    SwerveSubsytemKinematics kinematics;

    public SwerveSubsystem() {

        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(frontLeftWheelDistanceInches), Units.inchesToMeters(frontLeftWheelDistanceInches)), //Front Left
            new Translation2d(Units.inchesToMeters(frontRightWheelDistanceInches), Units.inchesToMeters(frontRightWheelDistanceInches)), //Front Right
            new Translation2d(Units.inchesToMeters(backLeftWheelDistanceInches), Units.inchesToMeters(backLeftWheelDistanceInches)), //Back Left
            new Translation2d(Units.inchesToMeters(backRightWheelDistanceInches), Units.inchesToMeters(backRightWheelDistanceInches)), //Back Right
        );
    }

    public void drive() {
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(testSpeedsX), Units.inchesToMeters(testSpeedsY), Units.degreesToRadians(testSpeedsDeg));

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
    }
    
}