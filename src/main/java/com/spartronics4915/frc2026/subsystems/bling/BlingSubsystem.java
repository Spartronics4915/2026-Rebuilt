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

        anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        anim1Chooser.setDefaultOption("RGB Fade", AnimationType.RgbFade);
        anim1Chooser.setDefaultOption("Single Fade", AnimationType.SingleFade);
        anim1Chooser.setDefaultOption("Strobe", AnimationType.Strobe);
        anim1Chooser.setDefaultOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", anim0Chooser);
        SmartDashboard.putData("Animation 1", anim1Chooser);
    }

    @Override
    public void blingSubsystemPeriodic() {
        final var anim0Selection = anim0Chooser.getSelected();
        if (anim0State != anim0Selection) {
            anim0State = anim0Selection;

            switch (anim0State) {
                default:
                case ColorFlow:
                    candle.setControl(
                        new ColorFlowAnimation(Slot0StartIdx, Slot0EndIdx).withSlot(0)
                            .withColor(Violet)
                    );
                    break;
                case Rainbow:
                    candle.setControl(
                        new RainbowAnimation(Slot0StartIdx, Slot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    candle.setControl(
                        new TwinkleAnimation(Slot0StartIdx, Slot0EndIdx).withSlot(0)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(Slot0StartIdx, Slot0EndIdx).withSlot(0)
                    );
                    break;
            }
        }

        final var anim1Selection = anim1Chooser.getSelected();
        if (anim1State != anim1Selection) {
            anim1State = anim1Selection;

            switch (anim1State) {
                default:
                case Larson:
                    candle.setControl(
                        new LarsonAnimation(Slot1StartIdx, Slot1EndIdx).withSlot(1)
                        .withColor(Red)
                    );
                    break;
                case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(Slot1StartIdx, Slot1EndIdx).withSlot(1)
                    );
                    break;
                case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(Slot1StartIdx,Slot1EndIdx).withSlot(1)
                            .withColor(Red)
                    );
                    break;
                case Strobe:
                    candle.setControl(
                        new StrobeAnimation(Slot1StartIdx, Slot1EndIdx).withSlot(1)
                            .withColor(Red)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(Slot1StartIdx, Slot1EndIdx).withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
            }
        }
    }

    //@Override
    //public void autonomousInit() {}

    //@Override
    //public void autonomousPeriodic() {}

    //@Override
    //public void teleopInit() {}

    //@Override
    //public void teleopPeriodic() {}

    //@Override
    //public void disabledInit() {}

    //@Override
    //public void disabledPeriodic() {}

    //@Override
    //public void testInit() {}

    //@Override
    //public void testPeriodic() {}

    //@Override
    //public void simulationInit() {}

    //@Override
    //public void simulationPeriodic() {}
}
