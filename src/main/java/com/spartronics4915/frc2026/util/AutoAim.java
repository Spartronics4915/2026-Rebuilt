package com.spartronics4915.frc2026.util;

import java.util.ArrayList;
import java.util.function.BiPredicate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoAim {
    private final int maxIterations;
    private double convergenceThreshold;
    private final Translation3d turretTransform;
    private final Rotation2d minAngle;
    private final Rotation2d maxAngle;
    private final double maxSpeed;
    private BiPredicate<Rotation2d, Double> collisionMap;

    private final double g = 9.81;

    /**
     * Constructor for AutoAim
     * 
     * @param maxIterations The max number of iterations used to calculate moving auto-aim and collision avoidance. Should be set high enough to
     *                      allow convergence, but low enough to prevent long runtimes in edge cases.
     * @param turretTransform The transform from the robot's center to the turret's center, in the robot's coordinate frame.
     * @param minAngle The minimum angle for auto-aim.
     * @param maxAngle The maximum angle for auto-aim.
     * @param maxSpeed The maximum speed for auto-aim.
     */
    public AutoAim(int maxIterations, double convergenceThreshold, Translation3d turretTransform, Rotation2d minAngle, Rotation2d maxAngle, double maxSpeed) {
        this.maxIterations = maxIterations;
        this.convergenceThreshold = convergenceThreshold;
        this.turretTransform = turretTransform;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.maxSpeed = maxSpeed;
    }

    /**
     * Output of auto-aim calculation, containing the yaw and pitch angles to aim at, as well as the time of flight for a projectile to
     * reach the target. Contains a recommended shot speed for the projectile for more consistent shots given current conditions.
     */
    public record AutoAimResult(Rotation2d yaw, Rotation2d pitch, double ToF, double recommendedShotSpeed) {}

    /**
     * Sets the collision map for the auto-aim system. Supplier should return true if the shot collides with an obstacle, and false if the shot does not.
     * 
     * @param collisionMap A BiPredicate that provides the collision map, or null for no collision map.
     */
    public void setCollisionMap(BiPredicate<Rotation2d, Double> collisionMap) {
        this.collisionMap = collisionMap;
    }

    public void setConvergenceThreshold(double convergenceThreshold) {
        this.convergenceThreshold = convergenceThreshold;
    }

    /**
     * Resolves the aim for a moving target / robot, taking into account the robot's current pose, field-relative speeds, target translation, and projectile shooting speed.
     * 
     * @param robotPose The current pose of the robot.
     * @param fieldSpeeds The current field-relative speeds of the robot.
     * @param targetTranslation The translation of the target relative to the field.
     * @param projectileSpeed The shooting speed of the projectile.
     * @return The result of the auto-aim calculation.
     */
    public AutoAimResult calculateDynamicAim(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d targetTranslation, double projectileSpeed) {
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
            Translation3d displacement = new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0).times(result.ToF());
            if (displacement.minus(prevDisplacement).getNorm() < convergenceThreshold) {
                return result;
            }
            prevDisplacement = displacement;
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
            if (collisionMap != null && collisionCheck(pitch, yaw, projectileSpeed, robotSpeeds)) {
                continue;
            }

            validSolutions.add(pitch);
        }

        if (validSolutions.isEmpty()) {
            return new AutoAimResult(yaw, null, -1, projectileSpeed);
        }

        // Select the best solution (usually lowest angle)
        Rotation2d selectedPitch = validSolutions.get(0);
        for (Rotation2d pitch : validSolutions) {
            if (Math.abs(pitch.getRadians()) < Math.abs(selectedPitch.getRadians())) {
                selectedPitch = pitch;
            }
        }

        double timeOfFlight = horizontalDistance / (projectileSpeed * selectedPitch.getCos());

        return new AutoAimResult(yaw, selectedPitch, timeOfFlight, projectileSpeed);
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

    private boolean collisionCheck(Rotation2d pitch, Rotation2d yaw, double projectileSpeed, ChassisSpeeds robotSpeeds) {
        double v_launch_xy = projectileSpeed * pitch.getCos();
        double v_launch_z = projectileSpeed * pitch.getSin();
        
        double v_launch_x = v_launch_xy * yaw.getCos();
        double v_launch_y = v_launch_xy * yaw.getSin();
        
        double v_ground_x = v_launch_x + robotSpeeds.vxMetersPerSecond;
        double v_ground_y = v_launch_y + robotSpeeds.vyMetersPerSecond;
        double v_ground_z = v_launch_z; // Assuming no vertical robot speed

        double ground_pitch = Math.atan2(v_ground_z, Math.hypot(v_ground_x, v_ground_y));
        double ground_speed = Math.sqrt(squared(v_ground_x) + squared(v_ground_y) + squared(v_ground_z));

        return collisionMap.test(new Rotation2d(ground_pitch), ground_speed);
    }
}
