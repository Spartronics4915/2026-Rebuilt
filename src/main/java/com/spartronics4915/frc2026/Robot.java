// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2026;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;
    private final BooleanPublisher hubEnabledPub = NetworkTableInstance.getDefault().getBooleanTopic("IO/Hub Enabled").publish();
    private final DoublePublisher timeUntilSwitchPub = NetworkTableInstance.getDefault().getDoubleTopic("IO/Time Left Until Switch").publish();
    private final BooleanPublisher currentAllianceSelectedPub = NetworkTableInstance.getDefault().getBooleanTopic("IO/Current Alliance Selected").publish();
    public boolean currentAllianceSelected = false;
    public static boolean hubEnabled;
    public static double timeUntilSwitch;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        NetworkTable metaData = NetworkTableInstance.getDefault().getTable("Metadata");
        metaData.getStringTopic("Git: SHA").publish().accept(BuildConstants.GIT_SHA);
        metaData.getStringTopic("Git: Branch").publish().accept(BuildConstants.GIT_BRANCH);
        metaData.getStringTopic("Git: Commit Date").publish().accept(BuildConstants.GIT_DATE);
        metaData.getStringTopic("Git: Build Date").publish().accept(BuildConstants.BUILD_DATE);
        metaData.getBooleanTopic("Git: Dirty").publish().accept(BuildConstants.DIRTY == 1);
        metaData.getDoubleTopic("Git: Revision").publish().accept(BuildConstants.GIT_REVISION);
        metaData.getBooleanTopic("Robot: IsSim").publish().accept(Robot.isSimulation());
        metaData.getStringTopic("DS: EventName").publish().accept(DriverStation.getEventName());

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
        // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods. This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        hubEnabledPub.set(true);
        hubEnabled = true;

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to continue until
        // interrupted by another command, remove this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

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
            } else if (matchTime > 105) { // (2:10 - 1:45) Shift 1
                hubEnabled = !currentAllianceSelected;
                timeUntilSwitch = matchTime - 105.0;
            } else if (matchTime > 80.0) {// (1:45 - 1:20) Shift 2
                hubEnabled = currentAllianceSelected;
                timeUntilSwitch = matchTime - 80.0;
            } else if (matchTime > 55.0) { // (1:20 - 0:55) Shift 3
                hubEnabled = !currentAllianceSelected;
                timeUntilSwitch = matchTime - 55.0;
            } else if (matchTime > 30.0) { // (0:55 - 0:30) Shift 4
                hubEnabled = currentAllianceSelected;
                timeUntilSwitch = matchTime - 30.0;
            } else if (matchTime > 0.0) { // (0:30 - 0:00) Endgame, both hubs are enabled
                hubEnabled = true;
                timeUntilSwitch = matchTime;
            } 

            hubEnabledPub.set(hubEnabled);
            timeUntilSwitchPub.set(timeUntilSwitch);
            currentAllianceSelectedPub.set(currentAllianceSelected);
        }
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
    public void simulationPeriodic() {}
}
