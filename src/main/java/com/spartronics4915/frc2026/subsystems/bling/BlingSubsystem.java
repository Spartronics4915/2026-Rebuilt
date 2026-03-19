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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BlingSubsystem extends SubsystemBase {
    private static final RGBWColor green = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor red = new RGBWColor(255, 255, 0, 0);
    private static final RGBWColor blue = new RGBWColor(0, 0, 255, 0);
    private static final RGBWColor purple = new RGBWColor(86, 26, 143, 0); //og = 186 112 255
    private static final RGBWColor yellow = new RGBWColor(246, 255, 0, 0);
    private static final RGBWColor white = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor violet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);

    private static final int slot0StartIdx = 0;
    private static final int slot0EndIdx = 40;

    private static final int slot1StartIdx = 41;
    private static final int slot1EndIdx = 70;

    private final CANdle candle = new CANdle(0, CANBus.roboRIO());

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

        cfg.LED.StripType = StripTypeValue.RGB;
        cfg.LED.BrightnessScalar = 1;

        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);

        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
        
        candle.setControl(new SolidColor(0, 70).withColor(blue));

        anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        anim0Chooser.addOption("Fire", AnimationType.Fire);
        anim0Chooser.addOption("Larson", AnimationType.Larson);
        anim0Chooser.addOption("RgbFade", AnimationType.RgbFade);
        anim0Chooser.addOption("SingleFade", AnimationType.SingleFade);
        anim0Chooser.addOption("Strobe", AnimationType.Strobe);

        anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        anim1Chooser.addOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", anim0Chooser);
        SmartDashboard.putData("Animation 1", anim1Chooser);
    }

    public void colorFlowAnimation() {
        candle.setControl(
            new ColorFlowAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                .withColor(yellow)
        );
    }

    public void fireAnimation() {
        candle.setControl(
            new FireAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                //.withDirection(AnimationDirectionValue.Backward)
                .withCooling(0.6)
                .withSparking(0.4)
        );
    }

    public void larsonAnimation() {
        candle.setControl(
            new LarsonAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
            .withColor(yellow)
        );
    }

    public void rainbowAnimation() {
        candle.setControl(
            new RainbowAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
        );
    }

    public void rgbFadeAnimation() {
        candle.setControl(
            new RgbFadeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
        );
    }

    public void singleFadeAnimation() {
        candle.setControl(
            new SingleFadeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                .withColor(yellow)
        );
    }

    public void strobeAnimation() {
        candle.setControl(
            new StrobeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                .withColor(yellow)
        );
    }

    public void positiveAnimation() {
        candle.setControl(new SolidColor(0, 70).withColor(green));
    }

    public void negativeAnimation() {
        candle.setControl(new SolidColor(0, 70).withColor(red));
    }

    public void spartronicsAnimation() {
        candle.setControl(new SolidColor(0, 9).withColor(blue));
        candle.setControl(new SolidColor(10, 19).withColor(yellow));
        candle.setControl(new SolidColor(20, 29).withColor(blue));
        candle.setControl(new SolidColor(30, 39).withColor(yellow));
        candle.setControl(new SolidColor(40, 49).withColor(blue));
        candle.setControl(new SolidColor(50, 59).withColor(yellow));
        candle.setControl(new SolidColor(60, 70).withColor(blue));
    }

    @Override
    public void periodic() {
        final var anim0Selection = anim0Chooser.getSelected();
        if (anim0State != anim0Selection) {
            anim0State = anim0Selection;

            switch (anim0State) {
                default:
                case ColorFlow:
                    candle.setControl(
                        new ColorFlowAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                            .withColor(yellow)
                            .withFrameRate(0.001)
                    );
                    break;
                case Rainbow:
                    candle.setControl(
                        new RainbowAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                    );
                    break;
                case Twinkle:
                    candle.setControl(
                        new TwinkleAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                    );
                    break;
                case TwinkleOff:
                    candle.setControl(
                        new TwinkleOffAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                            //.withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.6)
                            .withSparking(0.4)
                    );
                    break;
                case Larson:
                    candle.setControl(
                        new LarsonAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                        .withColor(green)
                    );
                    break;
                case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                    );
                    break;
                case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                            .withColor(green)
                    );
                    break;
                case Strobe:
                    candle.setControl(
                        new StrobeAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                            .withColor(green)
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
                        new LarsonAnimation(slot1StartIdx, slot1EndIdx).withSlot(1)
                        .withColor(red)
                    );
                    break;
                case RgbFade:
                    candle.setControl(
                        new RgbFadeAnimation(slot1StartIdx, slot1EndIdx).withSlot(1)
                    );
                    break;
                case SingleFade:
                    candle.setControl(
                        new SingleFadeAnimation(slot1StartIdx, slot1EndIdx).withSlot(1)
                            .withColor(red)
                    );
                    break;
                case Strobe:
                    candle.setControl(
                        new StrobeAnimation(slot1StartIdx, slot1EndIdx).withSlot(1)
                            .withColor(red)
                    );
                    break;
                case Fire:
                    candle.setControl(
                        new FireAnimation(slot1StartIdx, slot1EndIdx).withSlot(1)
                            .withDirection(AnimationDirectionValue.Backward)
                            .withCooling(0.4)
                            .withSparking(0.5)
                    );
                    break;
            }
        }
    }

    @Override
    public void simulationPeriodic() {}
}
