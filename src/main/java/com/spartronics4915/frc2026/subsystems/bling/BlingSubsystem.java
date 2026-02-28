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
    private static final RGBWColor green = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor red = new RGBWColor(255, 255, 0, 0);
    private static final RGBWColor blue = new RGBWColor(0, 0, 255, 0);
    private static final RGBWColor white = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor violet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);

    private static final int slot0StartIdx = 8;
    private static final int slot0EndIdx = 37;

    private static final int slot1StartIdx =38;
    private static final int slot1EndIdx = 67;

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
        
        candle.setControl(new SolidColor(0, 1).withColor(green));
        candle.setControl(new SolidColor(2, 3).withColor(white));
        candle.setControl(new SolidColor(4, 5).withColor(blue));
        candle.setControl(new SolidColor(6, 7).withColor(red));

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
                        new ColorFlowAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
                            .withColor(violet)
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
                case Fire:
                    candle.setControl(
                        new FireAnimation(slot0StartIdx, slot0EndIdx).withSlot(0)
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
