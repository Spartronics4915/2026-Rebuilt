package com.spartronics4915.frc2026.subsystems.mechanisms;

import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    public ClimberSubsystem() {
        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    public enum ClimberState {
        DOWN, UP
    }

}
