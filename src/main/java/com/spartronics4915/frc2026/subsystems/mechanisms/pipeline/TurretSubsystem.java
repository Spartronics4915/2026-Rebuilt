package com.spartronics4915.frc2026.subsystems.mechanisms.pipeline;

import com.spartronics4915.frc2026.util.ModeSwitchHandler;
import com.spartronics4915.frc2026.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase implements ModeSwitchInterface {
    
    public TurretSubsystem() {
        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    public enum TurretState {
        UNRESTRICTED, RESTRICTED
    }

}
