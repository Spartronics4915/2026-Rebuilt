package com.spartronics4915.frc2026.subsystems.bling;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


public class BlingSubsystem {
    private static final RGBWColor Green = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor Red = new RGBWColor(255, 255, 0, 0);
    private static final RGBWColor Blue = new RGBWColor(0, 0, 255, 0);
    private static final RGBWColor White = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor Violet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);

    private static final int Slot0StartIdx = 8;
    private static final int Slot0EndIdx = 37;

    private static final int Slot1StartIdx =38;
    private static final int Slot1EndIdx = 67;

    private final CANdle candle = new CANdle(1, CANBus.roboRIO());

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    private AnimationType anim0State = AnimationType.None;
    private AnimationType anim1State = AnimationType.None;

    private final SendableChooser<AnimationType> anim0Chooser = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> anim1Chooser = new SendableChooser<AnimationType>();

    public BlingSubsystem() {
        var cfg = new CANdleConfiguration();

        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;

        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);

        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
        
        candle.setControl(new SolidColor(0, 1).withColor(Green));
        candle.setControl(new SolidColor(2, 3).withColor(White));
        candle.setControl(new SolidColor(4, 5).withColor(Blue));
        candle.setControl(new SolidColor(6, 7).withColor(Red));

        anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        anim0Chooser.addOption("Fire", AnimationType.Fire);
    }
}
