package com.spartronics4915.frc2026.util.simulation;

import static com.spartronics4915.frc2026.Constants.AutoAimConstants.*;
import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.percentLoss;
import static com.spartronics4915.frc2026.Constants.SuperstructureConstants.shooterBaseTranslation;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import com.spartronics4915.frc2026.Constants.SwerveConstants.AutoConstants;
import com.spartronics4915.frc2026.subsystems.control.AutoAimController;
import com.spartronics4915.frc2026.subsystems.mechanisms.IntakeSubsystem;
import com.spartronics4915.frc2026.subsystems.swerve.SwerveSubsystem;
import com.spartronics4915.frc2026.util.control.AutoAim.AutoAimResult;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;
import com.spartronics4915.frc2026.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelSimHandler extends SubsystemBase {

    private FuelSim fuelSim;
    private SwerveSubsystem swerveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private AutoAimController autoAim;

    private int simulatedFuelStored = SIM_INITIAL_FUEL;
    private double loggedSimulatedFuelStored = SIM_INITIAL_FUEL;
    private double blueFuelScore;
    private double redFuelScore;
    private double lastSimulatedShotTimestamp = Double.NEGATIVE_INFINITY;

    private static Scope LOG = null;
    
    public FuelSimHandler(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, AutoAimController autoAim) {
        if (Robot.isReal()) return;

        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.autoAim = autoAim;


        // Main init
        fuelSim = new FuelSim("/Fuel Simulation");
        fuelSim.setSubticks(20);
        fuelSim.enableAirResistance();
        fuelSim.registerRobot(
            AutoConstants.robotWidth,
            AutoConstants.robotLength,
            Meters.of(0.20),
            () -> {
                return swerveSubsystem.getPose().rotateAround(
                    swerveSubsystem.getPose().getTranslation(),
                    Rotation2d.kCCW_90deg
                );
            },
            () -> ChassisSpeeds.fromRobotRelativeSpeeds(
                swerveSubsystem.getRobotVelocity(),
                swerveSubsystem.getPose().getRotation()));
        fuelSim.spawnStartingFuel();
        fuelSim.start();


        // Intake init
        double halfLength = AutoConstants.robotLength.in(Meters) / 2.0;
        double halfWidth = AutoConstants.robotWidth.in(Meters) / 2.0;

        // One intake mounted along the robot's -Y side.
        double intakeDepth = 0.18;

        fuelSim.registerIntake(
            -halfLength,
            halfLength,
            -halfWidth - intakeDepth,
            -halfWidth,
            this::isSimulationIntaking,
            this::intakeSimulatedFuel
        );


        // Log init
        LOG = Telemetry.scope("Simulation/FuelSim");
        LOG.debug.log("SimulatedFuelStored", loggedSimulatedFuelStored);
        LOG.debug.log("BlueFuelScore", blueFuelScore);
        LOG.debug.log("RedFuelScore", redFuelScore);
    }

    private void launchSimulatedFuel() {
        AutoAimResult result = autoAim.getLastResult();
        if (result == null || result.ToF() == -1) return;

        double launchSpeedMps = result.recommendedShotSpeed() / (1 - percentLoss);

        Angle pitch = Angle.ofBaseUnits(result.pitch().getRadians(), Radian);

        Rotation2d tempYaw = result.yaw()
            .minus(swerveSubsystem.getPose().getRotation())
            .plus(Rotation2d.kCCW_90deg);
        Angle yaw = Angle.ofBaseUnits(tempYaw.getRadians(), Radian);

        fuelSim.launchFuel(
            MetersPerSecond.of(launchSpeedMps),
            pitch,
            yaw,
            shooterBaseTranslation
        );

        simulatedFuelStored--;
        lastSimulatedShotTimestamp = Timer.getFPGATimestamp();
    }

    public void resetSimulatedFuel() {
        if (fuelSim == null) {
            return;
        }

        fuelSim.stop();
        fuelSim.clearFuel();
        FuelSim.Hub.BLUE_HUB.resetScore();
        FuelSim.Hub.RED_HUB.resetScore();
        fuelSim.spawnStartingFuel();
        simulatedFuelStored = SIM_INITIAL_FUEL;
        lastSimulatedShotTimestamp = Double.NEGATIVE_INFINITY;
        fuelSim.start();
    }

    public void intakeSimulatedFuel() {
        if (Robot.isSimulation() && simulatedFuelStored < SIM_FUEL_CAPACITY) {
            simulatedFuelStored++;
        }
    }

    public boolean isSimulationIntaking() {
        return Robot.isSimulation()
            && intakeSubsystem != null
            && intakeSubsystem.getSetpoint() > 5.0;
    }

    @Override
    public void simulationPeriodic() {
        fuelSim.updateSim();
        loggedSimulatedFuelStored = simulatedFuelStored;
        blueFuelScore = FuelSim.Hub.BLUE_HUB.getScore();
        redFuelScore = FuelSim.Hub.RED_HUB.getScore();

        if (!DriverStation.isEnabled() || !autoAim.isReadyToShoot() || simulatedFuelStored <= 0) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        if (now - lastSimulatedShotTimestamp < SIM_SHOT_INTERVAL_SECONDS) {
            return;
        }

        launchSimulatedFuel();
    }
}
