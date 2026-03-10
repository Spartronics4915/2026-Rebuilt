package com.spartronics4915.frc2026.util.control;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;

public class AutoAim {
    
    private final int maxIterations;
    private final int angleSearchSteps;
    private final double convergenceThreshold;
    private final Translation3d turretTransform;
    private final Rotation2d minAngle;
    private final Rotation2d maxAngle;
    private final double maxSpeed;
    private final double lookaheadTime;
    
    public interface CollisionMap {
        boolean test(Rotation2d groundPitch, double groundSpeed);
    }
    
    private CollisionMap collisionMap;
    private CollisionMap idealVelocityCollisionMap;

    private final double g = 9.81;

    /**
     * Constructor for AutoAim
     * 
     * @param maxIterations The max number of iterations used to calculate moving auto-aim and collision avoidance.
     * @param angleSearchSteps The number of steps to use when searching for the ideal velocity angle.
     * @param convergenceThreshold The threshold for convergence in meters for the auto-aim calculation.
     * @param turretTransform The transform from the robot's center to the turret's center.
     * @param minAngle The minimum angle for auto-aim.
     * @param maxAngle The maximum angle for auto-aim.
     * @param maxSpeed The maximum speed for auto-aim.
     * @param lookaheadTime Amount of time auto-aim looks ahead to calculate omega for pitch and yaw.
     * @param collisionMap A predicate that provides the collision map, or null for no collision map.
     * @param idealVelocityCollisionMap A predicate that provides the collision map with extra padding.
     */
    public AutoAim(int maxIterations, int angleSearchSteps, double convergenceThreshold, Translation3d turretTransform, Rotation2d minAngle, Rotation2d maxAngle, double maxSpeed, double lookaheadTime, CollisionMap collisionMap, CollisionMap idealVelocityCollisionMap) {
        this.maxIterations = maxIterations;
        this.angleSearchSteps = angleSearchSteps;
        this.convergenceThreshold = convergenceThreshold;
        this.turretTransform = turretTransform;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxSpeed = maxSpeed;
        this.lookaheadTime = lookaheadTime;
        this.collisionMap = collisionMap;
        this.idealVelocityCollisionMap = idealVelocityCollisionMap;
    }

    /**
     * Dynamically update the collision maps.
     * 
     * @param collisionMap A predicate that provides the new collision map, or null for no collision map.
     * @param idealVelocityCollisionMap A predicate that provides the new ideal velocity collision map.
     */
    public void setCollisionMap(CollisionMap collisionMap, CollisionMap idealVelocityCollisionMap) {
        this.collisionMap = collisionMap;
        this.idealVelocityCollisionMap = idealVelocityCollisionMap;
    }

    /**
     * Output of the auto-aim calculation.
     *
     * <ul>
     *   <li>{@code yaw} — the horizontal angle to aim at ({@link Rotation2d}).</li>
     *   <li>{@code yawOmega} — the angular velocity of the yaw ({@link AngularVelocity}).</li>
     *   <li>{@code pitch} — the vertical angle to aim at ({@link Rotation2d}); may be {@code null} when unsolvable.</li>
     *   <li>{@code pitchOmega} — the angular velocity of the pitch ({@link AngularVelocity}).</li>
     *   <li>{@code ToF} — time of flight in seconds, or {@code -1} when the solver signals the shot cannot land.</li>
     *   <li>{@code recommendedShotSpeed} — a recommended projectile speed in meters per second, or {@code -1} when not applicable.</li>
     *   <li>{@code requiresIdealSpeed} — {@code true} when the shot is impossible at the provided flywheel speed (i.e. the returned
     *       {@code yaw}/{@code pitch} require a higher/ideal speed); {@code false} when the shot is achievable with the supplied speed.</li>
     * </ul>
     */
    public record AutoAimResult(Rotation2d yaw, AngularVelocity yawOmega, Rotation2d pitch, AngularVelocity pitchOmega, double ToF, double recommendedShotSpeed, boolean requiresIdealSpeed) {
        
        public static AutoAimResult withoutOmega(Rotation2d yaw, Rotation2d pitch, double ToF, double recommendedShotSpeed, boolean requiresIdealSpeed) {
            return new AutoAimResult(yaw, DegreesPerSecond.of(0), pitch, DegreesPerSecond.of(0), ToF, recommendedShotSpeed, requiresIdealSpeed);
        }

        public AutoAimResult withYawOmega(AngularVelocity newYawOmega) {
            return new AutoAimResult(yaw, newYawOmega, pitch, pitchOmega, ToF, recommendedShotSpeed, requiresIdealSpeed);
        }

        public AutoAimResult withPitchOmega(AngularVelocity newPitchOmega) {
            return new AutoAimResult(yaw, yawOmega, pitch, newPitchOmega, ToF, recommendedShotSpeed, requiresIdealSpeed);
        }
    }


    /**
     * Resolves the aim for a moving target / robot, taking into account the robot's current pose, field-relative speeds, target translation, and projectile shooting speed.
     * 
     * @param robotPose The current pose of the robot.
     * @param fieldSpeeds The current field-relative speeds of the robot.
     * @param targetTranslation The translation of the target relative to the field.
     * @param projectileSpeed The shooting speed of the projectile.
     * @param processingCompensation The compensation (in seconds) for processing latency.
     * @return The result of the auto-aim calculation.
     */
    public AutoAimResult calculateDynamicAim(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d targetTranslation, double projectileSpeed, double processingCompensation) {
        double turretVxRobot = -fieldSpeeds.omegaRadiansPerSecond * turretTransform.getY();
        double turretVyRobot = fieldSpeeds.omegaRadiansPerSecond * turretTransform.getX();
        
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();
        
        double turretVxField = turretVxRobot * cos - turretVyRobot * sin;
        double turretVyField = turretVxRobot * sin + turretVyRobot * cos;

        ChassisSpeeds speeds = new ChassisSpeeds(
            fieldSpeeds.vxMetersPerSecond + turretVxField,
            fieldSpeeds.vyMetersPerSecond + turretVyField,
            fieldSpeeds.omegaRadiansPerSecond
        );

        Translation3d prevDisplacement = new Translation3d();
        AutoAimResult result = null;
        for (int i = 0; i < maxIterations; i++) {
            Translation3d virtualTarget = targetTranslation.minus(prevDisplacement);
            result = calculateStaticAim(robotPose, virtualTarget, projectileSpeed, speeds);
            if (result == null || result.ToF() == -1) {
                return result;
            }
            Translation3d displacement = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0).times(result.ToF() + processingCompensation);
            if (i > 0 && displacement.minus(prevDisplacement).getNorm() < convergenceThreshold) {
                break;
            }
            prevDisplacement = displacement;
        }

        if (result == null) {
            return null;
        }

        AutoAimResult lookaheadResult = calculateStaticAim(
            robotPose,
            targetTranslation.minus(
                new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0)
                    .times(result.ToF() + processingCompensation + lookaheadTime)
            ),
            projectileSpeed,
            speeds
        );

        if (lookaheadResult != null) {
            result = result.withYawOmega(
                DegreesPerSecond.of(
                    (lookaheadResult.yaw().minus(result.yaw()).getDegrees() / lookaheadTime) 
                    - Math.toDegrees(fieldSpeeds.omegaRadiansPerSecond)
                )
            );

            if (lookaheadResult.pitch() != null && result.pitch() != null) {
                result = result.withPitchOmega(
                    DegreesPerSecond.of(
                        lookaheadResult.pitch().minus(result.pitch()).getDegrees() / lookaheadTime
                    )
                );
            }
        }

        return result;
    }

    /**
     * Resolves the aim for a moving target / robot, taking into account the robot's current pose, target translation, and projectile shooting speed.
     * 
     * @param robotPose The current pose of the robot.
     * @param targetTranslation The translation of the target relative to the field.
     * @param projectileSpeed The shooting speed of the projectile.
     * @return The result of the auto-aim calculation.
     */
    public AutoAimResult calculateStaticAim(Pose2d robotPose, Translation3d targetTranslation, double projectileSpeed) {
        return calculateStaticAim(robotPose, targetTranslation, projectileSpeed, new ChassisSpeeds());
    }

    /**
     * Utility function to calculate static aim but collision check with robot velocity.
     * 
     * @param robotPose The current pose of the robot.
     * @param targetTranslation The translation of the target relative to the field.
     * @param projectileSpeed The shooting speed of the projectile.
     * @param robotSpeeds The speeds of the robot (including turret velocity relative to field).
     * @return The result of the auto-aim calculation.
     */
    private AutoAimResult calculateStaticAim(Pose2d robotPose, Translation3d targetTranslation, double projectileSpeed, ChassisSpeeds robotSpeeds) {
        if (projectileSpeed < 0.0) {
            return null;
        }

        // Calculate robot-to-turret transform in field space
        Translation3d turretToField = new Translation3d(robotPose.getX(), robotPose.getY(), 0.0)
            .plus(turretTransform.rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        
        // Vector from turret to target
        Translation3d relativeTransform3d = targetTranslation.minus(turretToField);
        
        double horizontalDistance = relativeTransform3d.toTranslation2d().getNorm();
        double verticalHeight = relativeTransform3d.getZ();

        // Calculate Yaw
        Rotation2d yaw = new Rotation2d(Math.atan2(relativeTransform3d.getY(), relativeTransform3d.getX()));

        // Physics constants
        double g_x2 = g * squared(horizontalDistance);
        double v2 = squared(projectileSpeed);
        
        // coefficients for a*tan(theta)^2 + b*tan(theta) + c = 0
        double a = g_x2 / (2 * v2);
        double b = -horizontalDistance;
        double c = verticalHeight + a;

        double[] tan_thetas = quadraticSolver(a, b, c);

        // Filter valid solutions
        ArrayList<Rotation2d> validSolutions = new ArrayList<>();
        
        for (double tan_t : tan_thetas) {
            Rotation2d pitch = new Rotation2d(Math.atan(tan_t));
            // Check min/max angle constraints
            if (pitch.getRadians() < minAngle.getRadians() || pitch.getRadians() > maxAngle.getRadians()) {
                continue;
            }
            
            // Check collision map if it exists and if it collides
            if (collisionMap != null && collisionCheck(collisionMap, pitch, yaw, projectileSpeed, robotSpeeds)) {
                continue;
            }

            validSolutions.add(pitch);
        }

        double recommendedSpeed = -1;
        Rotation2d recommendedPitch = null;
        
        for (
            Rotation2d angle=minAngle; 
            angle.getRadians() < maxAngle.getRadians(); 
            angle = angle.plus(maxAngle.minus(minAngle).div(angleSearchSteps))
        ) {
            // Calculate height difference between straight-line path and target at distance x
            // dist = x * tan(theta) - (targetZ - turretZ)
            double distAboveHub = horizontalDistance * angle.getTan() - (targetTranslation.getZ() - turretToField.getZ());

            // If we are aiming lower than the target, no velocity can get us there (ignoring lift)
            if (distAboveHub <= 0) continue;

            // v^2 = g * x^2 / (2 * cos^2(theta) * distAboveHub)
            double vSquared = (g * squared(horizontalDistance)) / (2 * squared(angle.getCos()) * distAboveHub);
            double v = Math.sqrt(vSquared);

            if (v > maxSpeed) continue;

            // When searching for an ideal velocity, use padding in the collision map.
            if (idealVelocityCollisionMap != null && collisionCheck(idealVelocityCollisionMap, angle, yaw, v, robotSpeeds)) continue;

            recommendedSpeed = v;
            recommendedPitch = angle;
            break;
        }

        if (recommendedSpeed == -1 || recommendedPitch == null) {
            return AutoAimResult.withoutOmega(yaw, null, -1, -1, false);
        }

        if (validSolutions.isEmpty()) {
            double timeOfFlight = horizontalDistance / (recommendedSpeed * recommendedPitch.getCos());
            return AutoAimResult.withoutOmega(yaw, recommendedPitch, timeOfFlight, recommendedSpeed, true);
        }

        // Select the best solution (usually lowest angle)
        Rotation2d selectedPitch = validSolutions.get(0);
        for (Rotation2d pitch : validSolutions) {
            if (Math.abs(pitch.getRadians()) < Math.abs(selectedPitch.getRadians())) {
                selectedPitch = pitch;
            }
        }

        double timeOfFlight = horizontalDistance / (projectileSpeed * selectedPitch.getCos());

        return AutoAimResult.withoutOmega(yaw, selectedPitch, timeOfFlight, recommendedSpeed, false);
    }

    private double[] quadraticSolver(double a, double b, double c) {
        final double eps = 1e-9;

        // If a is (near) zero, fall back to linear equation: b*x + c = 0
        if (Math.abs(a) < eps) {
            if (Math.abs(b) < eps) {
                // Degenerate: a and b are ~0. Treat as no solution.
                return new double[0];
            }
            return new double[]{-c / b};
        }

        double discriminant = squared(b) - 4 * a * c;
        if (discriminant < -eps) {
            return new double[0]; // No real solutions
        } else if (Math.abs(discriminant) <= eps) {
            return new double[]{-b / (2 * a)}; // One real solution (within tolerance)
        } else {
            double sqrtDiscriminant = Math.sqrt(discriminant);
            return new double[]{
                (-b + sqrtDiscriminant) / (2 * a),
                (-b - sqrtDiscriminant) / (2 * a)
            }; // Two real solutions
        }
    }

    private double squared(double value) {
        return value * value;
    }

    private boolean collisionCheck(CollisionMap map, Rotation2d pitch, Rotation2d yaw, double projectileSpeed, ChassisSpeeds robotSpeeds) {
        double v_launch_xy = projectileSpeed * pitch.getCos();
        double v_launch_z = projectileSpeed * pitch.getSin();
        
        double v_launch_x = v_launch_xy * yaw.getCos();
        double v_launch_y = v_launch_xy * yaw.getSin();
        
        double v_ground_x = v_launch_x + robotSpeeds.vxMetersPerSecond;
        double v_ground_y = v_launch_y + robotSpeeds.vyMetersPerSecond;
        double v_ground_z = v_launch_z; // Assuming no vertical robot speed

        double ground_pitch = Math.atan2(v_ground_z, Math.hypot(v_ground_x, v_ground_y));
        double ground_speed = Math.sqrt(squared(v_ground_x) + squared(v_ground_y) + squared(v_ground_z));

        return map.test(new Rotation2d(ground_pitch), ground_speed);
    }
}
