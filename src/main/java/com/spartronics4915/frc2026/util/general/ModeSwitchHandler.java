package com.spartronics4915.frc2026.util.general;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public final class ModeSwitchHandler {
    public interface ModeSwitchInterface {
        /**
         * This method will trigger whenever the robot is enabled or switches mode (e.g., from autonomous to teleoperated) 
         */
        default void onModeSwitch(){}

        /**
         * This method will trigger whenever the robot is disabled
         */
        default void onDisable(){}
    }

    public static void EnableModeSwitchHandler(ModeSwitchInterface... modeSwitches) {

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            for(var index : modeSwitches){
                index.onDisable();
            }
        }).ignoringDisable(true));

        RobotModeTriggers.teleop()
            .or(RobotModeTriggers.autonomous())
            .or(RobotModeTriggers.test())
            .onTrue(
                Commands.runOnce(() -> {
                    for(var index : modeSwitches){
                        index.onModeSwitch();
                    }}
                ).ignoringDisable(true)
            );
    }
}
