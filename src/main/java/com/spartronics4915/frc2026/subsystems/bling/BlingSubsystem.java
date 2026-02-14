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

    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }
}
