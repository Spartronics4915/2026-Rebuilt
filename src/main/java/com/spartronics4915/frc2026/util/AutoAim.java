package com.spartronics4915.frc2026.util;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

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
    private BooleanSupplier collisionMap;

    private final double g = 9.81;

    /**
     * Constructor for AutoAim
     * 
     * @param maxIterations The max number of iterations used to calculate moving auto-aim and collision avoidance. Should be set high enough to
     *                      allow convergence, but low enough to prevent long runtimes in edge cases.
     * @param turretTransform The transform from the robot's center to the turret's center, in the robot's coordinate frame.
     * @param minAngle The minimum angle for auto-aim.
     * @param maxAngle The maximum angle for auto-aim.
     */
    public AutoAim(int maxIterations, double convergenceThreshold, Translation3d turretTransform, Rotation2d minAngle, Rotation2d maxAngle) {
        this.maxIterations = maxIterations;
        this.convergenceThreshold = convergenceThreshold;
        this.turretTransform = turretTransform;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    /**
     * Output of auto-aim calculation, containing the yaw and pitch angles to aim at, as well as the time of flight for a projectile to
     * reach the target. Contains a recommended shot speed for the projectile for more consistent shots given current conditions.
     */
    public record AutoAimResult(Rotation2d yaw, Rotation2d pitch, double ToF, double recommendedShotSpeed) {}

    /**
     * Sets the collision map for the auto-aim system. Supplier should return true if the shot collides with an obstacle, and false if the shot does not.
     * 
     * @param collisionMap A BooleanSupplier that provides the collision map, or null for no collision map.
     */
    public void setCollisionMap(BooleanSupplier collisionMap) {
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
            result = calculateStaticAim(robotPose, virtualTarget, projectileSpeed);
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
        if (projectileSpeed < 0.0) {
            return null;
        }

        Translation3d robotToTurret = new Translation3d(robotPose.getX(), robotPose.getY(), 0.0)
            .plus(turretTransform.rotateBy(new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        
        Translation3d relativeTransform3d = targetTranslation.minus(robotToTurret);
        
        double horizontalDistance = relativeTransform3d.toTranslation2d().getNorm();
        double verticalHeight = relativeTransform3d.getZ();

        Rotation2d yaw = new Rotation2d(Math.atan2(relativeTransform3d.getY(), relativeTransform3d.getX()));

        double[] tan_theta = quadraticSolver(
            (g*squared(horizontalDistance))/(2*squared(projectileSpeed)),
            -horizontalDistance,
            verticalHeight + (g*squared(horizontalDistance))/(2*squared(projectileSpeed))
        );

        Rotation2d[] theta = Arrays.stream(tan_theta)
            .map(Math::atan)
            .mapToObj(Rotation2d::new)
            .toArray(Rotation2d[]::new);

        theta = discardExtraneous(theta, minAngle, maxAngle);

        Rotation2d selectedPitch;
        if (theta.length == 0) {
            selectedPitch = null;
        } else if (theta.length == 1) {
            selectedPitch = theta[0];
        } else {
            selectedPitch = theta[0].getDegrees() < theta[1].getDegrees() ? theta[0] : theta[1];
        }

        double t = selectedPitch == null ? -1 : horizontalDistance / (projectileSpeed * Math.cos(selectedPitch.getRadians()));

        return new AutoAimResult(yaw, selectedPitch, t, projectileSpeed);
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

    private Rotation2d[] discardExtraneous(Rotation2d[] solutions, Rotation2d min, Rotation2d max) {
        return Arrays.stream(solutions)
                     .filter(solution -> solution.getRadians() >= min.getRadians() && solution.getRadians() <= max.getRadians())
                     .toArray(Rotation2d[]::new);
    }
}
