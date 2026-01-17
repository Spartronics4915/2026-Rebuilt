import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;

public class OdometrySubsystem {

    SwerveSubsystemOdometry odometry;
    Gyroscope gyro;
    SwerveModule[] swerveModules;

    public OdometrySubsystem() {

        swerveModules = new SwerveModule[4];

        gyro = new Gyroscope();

        odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getAngle(),
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}, // FrontLeft, FrontRight, BackLeft, BackRight
            new Pose2d(0,0,new Rotation2d()) // x=0 y=0 heading=0
        );
    }

    public void drive() {

        swerveModule[0].setState(swerveModuleStates[0]);
        swerveModule[1].setState(swerveModuleStates[1]);
        swerveModule[2].setState(swerveModuleStates[2]);
        swerveModule[3].setState(swerveModuleStates[3]);
    }

    @Override
    public void periodic() {

        odometry.update(gyro.getAngle(), getCurrentSwerveModulePositions());
    }
    
}
