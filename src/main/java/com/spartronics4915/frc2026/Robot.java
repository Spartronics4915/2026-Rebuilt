// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import com.spartronics4915.frc2026.autos.Autos;
import com.spartronics4915.frc2026.util.logging.Telemetry;
import com.spartronics4915.frc2026.util.logging.Telemetry.Scope;

import au.grapplerobotics.CanBridge;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.util.List;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private static final Scope MATCH_LOG = Telemetry.scope("Match");
    private static final Scope PERFORMANCE_LOG = Telemetry.scope("Performance");

    private Command autonomousCommand;

    private final RobotContainer robotContainer;
    private final PerformanceMetrics performanceTelemetry = new PerformanceMetrics();
    private long lastCanStatusUpdateUs;
    private String shiftName = "Unknown";
    public boolean currentAllianceSelected = false;
    
    public static boolean hubEnabled;
    public static double timeUntilSwitch;
    public static boolean isPureTeleop = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        CanBridge.runTCP();
    }

    @Override
    public void robotInit() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        performanceTelemetry.beginLoop();
        long schedulerStartUs = RobotController.getFPGATime();
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        performanceTelemetry.SchedulerDurationUs = RobotController.getFPGATime() - schedulerStartUs;
        performanceTelemetry.VisionDurationUs = robotContainer.visionSubsystem.getPeriodicDurationUs();

        long nowUs = RobotController.getFPGATime();
        if (nowUs - lastCanStatusUpdateUs >= 1_000_000) {
            var canStatus = Constants.GeneralConstants.CAN_BUS.getStatus();
            performanceTelemetry.CanBusStatusOk = canStatus.Status.isOK();
            performanceTelemetry.CanBusUtilization = canStatus.BusUtilization;
            performanceTelemetry.updateGarbageCollection();
            lastCanStatusUpdateUs = nowUs;
        }
        updateMatchTelemetry();
        outputPerformanceTelemetry();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        Autos.surveyMode = false;
        autonomousCommand = robotContainer.getAutonomousCommand();

        shiftName = "Auto";
        hubEnabled = true;
        timeUntilSwitch = 999.0;

        // schedule the autonomous command (example)
        robotContainer.swerveSubsystem.configureStdDevsEnabled();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        robotContainer.swerveSubsystem.configureStdDevsEnabled();

        // If match time is near zero (or negative), we are likely in pure Teleop mode (counting up)
        // In practice mode or real matches, teleop starts with a high number (e.g. 135) and counts down.
        isPureTeleop = DriverStation.getMatchTime() < 10.0;
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        if (isPureTeleop) {
            hubEnabled = true;
            timeUntilSwitch = 999.0;
            currentAllianceSelected = false;

            shiftName = "Pure Teleop";
            return;
        }

        // Hub enable/disable flashing logic
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            if (gameData.charAt(0) == 'R') {
                currentAllianceSelected = true;
            } else if (gameData.charAt(0) == 'B') {
                currentAllianceSelected = false;
            }

            currentAllianceSelected ^= DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

            double matchTime = DriverStation.getMatchTime();

            if (matchTime > 130.0) { // (2:20 - 2:10) Transition shift, both hubs are enabled
                hubEnabled = true;
                timeUntilSwitch = matchTime - 130.0 + (currentAllianceSelected ? 0 : 25.0);
                shiftName = "Transition Shift";
            } else if (matchTime > 105) { // (2:10 - 1:45) Shift 1
                hubEnabled = !currentAllianceSelected;
                timeUntilSwitch = matchTime - 105.0;
                shiftName = "Shift 1";
            } else if (matchTime > 80.0) {// (1:45 - 1:20) Shift 2
                hubEnabled = currentAllianceSelected;
                timeUntilSwitch = matchTime - 80.0;
                shiftName = "Shift 2";
            } else if (matchTime > 55.0) { // (1:20 - 0:55) Shift 3
                hubEnabled = !currentAllianceSelected;
                timeUntilSwitch = matchTime - 55.0;
                shiftName = "Shift 3";
            } else if (matchTime > 30.0) { // (0:55 - 0:30) Shift 4
                hubEnabled = currentAllianceSelected;
                timeUntilSwitch = matchTime - 30.0;
                shiftName = "Shift 4";
            } else if (matchTime > 0.0) { // (0:30 - 0:00) Endgame, both hubs are enabled
                hubEnabled = true;
                timeUntilSwitch = matchTime;
                shiftName = "Endgame";
            } 

        } else {
            hubEnabled = false;
            timeUntilSwitch = 999.0;
            currentAllianceSelected = false;
            shiftName = "No Game Data";
        }
    }

    /** Measures the complete WPILib main periodic cycle, not the jitter between cycle starts. */
    @Override
    protected void loopFunc() {
        long cycleStartUs = RobotController.getFPGATime();
        try {
            super.loopFunc();
        } finally {
            performanceTelemetry.recordPeriodicCycleDuration(
                RobotController.getFPGATime() - cycleStartUs);
        }
    }

    private void updateMatchTelemetry() {
        MATCH_LOG.critical.log("SampleTimestampUs", RobotController.getFPGATime());
        MATCH_LOG.critical.log("MatchTimeSeconds", DriverStation.getMatchTime());
        MATCH_LOG.critical.log("HubEnabled", hubEnabled);
        MATCH_LOG.critical.log("TimeUntilSwitchSeconds", timeUntilSwitch);
        MATCH_LOG.critical.log("CurrentAllianceSelected", currentAllianceSelected);
        MATCH_LOG.critical.log("ShiftName", shiftName);
        MATCH_LOG.info.log("EventName", DriverStation.getEventName());
        MATCH_LOG.info.log("GitSha", BuildConstants.GIT_SHA);
        MATCH_LOG.info.log("GitBranch", BuildConstants.GIT_BRANCH);
        MATCH_LOG.info.log("GitCommitDate", BuildConstants.GIT_DATE);
        MATCH_LOG.info.log("BuildDate", BuildConstants.BUILD_DATE);
        MATCH_LOG.info.log("GitDirty", BuildConstants.DIRTY == 1);
        MATCH_LOG.info.log("GitRevision", BuildConstants.GIT_REVISION);
        MATCH_LOG.info.log("IsSimulation", Robot.isSimulation());
    }

    private void outputPerformanceTelemetry() {
        PERFORMANCE_LOG.critical.log("SampleTimestampUs", performanceTelemetry.SampleTimestampUs);
        PERFORMANCE_LOG.critical.log("MainStartIntervalUs", performanceTelemetry.MainStartIntervalUs);
        PERFORMANCE_LOG.critical.log("PeriodicCycleDurationUs", performanceTelemetry.PeriodicCycleDurationUs);
        PERFORMANCE_LOG.critical.log("SchedulerDurationUs", performanceTelemetry.SchedulerDurationUs);
        PERFORMANCE_LOG.critical.log("MissedDeadlineCount", performanceTelemetry.MissedDeadlineCount);
        PERFORMANCE_LOG.critical.log("CanBusStatusOk", performanceTelemetry.CanBusStatusOk);
        PERFORMANCE_LOG.info.log("VisionDurationUs", performanceTelemetry.VisionDurationUs);
        PERFORMANCE_LOG.info.log("GarbageCollectionCount", performanceTelemetry.GarbageCollectionCount);
        PERFORMANCE_LOG.info.log("GarbageCollectionDurationMs", performanceTelemetry.GarbageCollectionDurationMs);
        PERFORMANCE_LOG.info.log("LoggingOffsetMs", 0.0);
        PERFORMANCE_LOG.info.log("CanBusUtilization", performanceTelemetry.CanBusUtilization);
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        
    }

    private static final class PerformanceMetrics {
        private static final long LOOP_PERIOD_US = 20_000;
        private static final List<GarbageCollectorMXBean> GC_BEANS =
            ManagementFactory.getGarbageCollectorMXBeans();

        long SampleTimestampUs;
        long MainStartIntervalUs;
        long PeriodicCycleDurationUs;
        long SchedulerDurationUs;
        long VisionDurationUs;
        long MissedDeadlineCount;
        long GarbageCollectionCount;
        long GarbageCollectionDurationMs;
        double CanBusUtilization;
        boolean CanBusStatusOk;
        private long previousMainStartUs;

        void beginLoop() {
            long now = RobotController.getFPGATime();
            if (previousMainStartUs != 0) MainStartIntervalUs = now - previousMainStartUs;
            previousMainStartUs = now;
            SampleTimestampUs = now;
        }

        void recordPeriodicCycleDuration(long durationUs) {
            PeriodicCycleDurationUs = durationUs;
            if (durationUs > LOOP_PERIOD_US) MissedDeadlineCount++;
        }

        void updateGarbageCollection() {
            long count = 0;
            long durationMs = 0;
            for (GarbageCollectorMXBean bean : GC_BEANS) {
                if (bean.getCollectionCount() >= 0) count += bean.getCollectionCount();
                if (bean.getCollectionTime() >= 0) durationMs += bean.getCollectionTime();
            }
            GarbageCollectionCount = count;
            GarbageCollectionDurationMs = durationMs;
        }
    }

}
