package com.spartronics4915.frc2026.subsystems.bling;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.spartronics4915.frc2026.Constants;

public class BlingSubsystem {
    private final CANdle candle = new CANdle(Constants.BlingConstants.CANdleID, "rio");
    private final int LedCount = 300;
    private XboxController joystick;

    private Animation toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    
    private AnimationTypes currentAnimation;
}
