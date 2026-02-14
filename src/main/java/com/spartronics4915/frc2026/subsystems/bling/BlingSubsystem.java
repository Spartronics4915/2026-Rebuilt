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

    public BlingSubsystem(XboxController joy) {
        this.joystick = joy;
        changeAnimation(AnimationTypes.SetAll);

        CANdleConfiguration configAll = new CANdleConfiguration();

        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
    }

    public void incrementAnimation() {
        switch(currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Fire); break;
            case Larson: changeAnimation(AnimationTypes.Larson); break;
            case Rainbow: changeAnimation(AnimationTypes.Rainbow); break;
            case RgbFade: changeAnimation(AnimationTypes.RgbFade); break;
            case SingleFade: changeAnimation(AnimationTypes.SingleFade); break;
            case Strobe: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.TwinkleOff); break;
            case SetAll: changeAnimation(AnimationTypes.SetAll); break;
        }
    }

    public void decrementAnimation() {
        switch(currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.ColorFlow); break;
            case Fire: changeAnimation(AnimationTypes.Fire); break;
            case Larson: changeAnimation(AnimationTypes.Larson); break;
            case Rainbow: changeAnimation(AnimationTypes.Rainbow); break;
            case RgbFade: changeAnimation(AnimationTypes.RgbFade); break;
            case SingleFade: changeAnimation(AnimationTypes.SingleFade); break;
            case Strobe: changeAnimation(AnimationTypes.Strobe); break;
            case Twinkle: changeAnimation(AnimationTypes.Twinkle); break;
            case TwinkleOff: changeAnimation(AnimationTypes.TwinkleOff); break;
            case SetAll: changeAnimation(AnimationTypes.SetAll); break;
        }
    }

    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    //Wrappers to access CANdle from the subsystem
    public double getVbat() { return candle.getBusVoltage(); }
    public double get5V() { return candle.get5VRailVoltage(); }
    public double getCurrent() { return candle.getCurrent(); }
    public double getTemperature() { return candle.getTemperature(); }
    public void configBrightness(double percent) { candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        currentAnimation = toChange;

        switch(toChange) {
            case ColorFlow:
                toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, Constants.BlingConstants.LedCount, Direction.Forward);
                break;
            case Fire:
                toAnimate = new FireAnimation(0.5, 0.7, Constants.BlingConstants.LedCount, 0.7, 0.5);
                break;
            case Larson:
                toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, Constants.BlingConstants.LedCount, BounceMode.Front,3);
                break;
            case Rainbow:
                toAnimate = new RainbowAnimation(1, 0.1, Constants.BlingConstants.LedCount);
                break;
            case RgbFade:
                toAnimate = new RgbFadeAnimation(0.7, 0.4, Constants.BlingConstants.LedCount);
                break;
            case SingleFade:
                toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.BlingConstants.LedCount);
                break;
            case Strobe:
                toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, Constants.BlingConstants.LedCount);
                break;
            case Twinkle:
                toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, Constants.BlingConstants.LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, Constants.BlingConstants.LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                toAnimate = null;
                break;
        }
    System.out.println("Changed to " + currentAnimation.toString());
    }
}
