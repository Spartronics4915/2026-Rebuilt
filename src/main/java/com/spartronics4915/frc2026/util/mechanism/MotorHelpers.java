// Utility class for logging Krakens to SmartDashboard

package com.spartronics4915.frc2026.util.mechanism;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;

public class MotorHelpers {
    public class CTRE {
        public static class LoggedTalonFX extends TalonFX {
            private int canID;
            private double p;
            private double i;
            private double d;
            private double v;
            private double a;
            private double s;
            private boolean following;
            private LoggedTrapezoidProfile profile;
            private LoggedSlewRateLimiter slewLimiter;
            private double maxVelocity;
            private double maxAcceleration;
            private DoubleSupplier getSetpoint;
            private DoubleConsumer setSetpoint;

            public LoggedTalonFX(int deviceId, CANBus canbus) {
                super(deviceId, canbus);
                this.canID = deviceId;
            }

            public void addProfile(LoggedTrapezoidProfile profile) {
                this.profile = profile;
                maxVelocity = profile.constraints.maxVelocity;
                maxAcceleration = profile.constraints.maxAcceleration;
            }

            public void addProfile(LoggedSlewRateLimiter limiter) {
                this.slewLimiter = limiter;
            }

            public LoggedTrapezoidProfile getProfile() {
                return profile;
            }

            public void addSetpoint(DoubleSupplier getSetpoint, DoubleConsumer setSetpoint) {
                this.getSetpoint = getSetpoint;
                this.setSetpoint = setSetpoint;
            }

            private void applyPID() {
                super.getConfigurator().apply(
                    new SlotConfigs()
                    .withKP(p)
                    .withKI(i)
                    .withKD(d)
                    .withKV(v)
                    .withKA(a)
                    .withKS(s)
                );
            }

            private void applyProfileConstraints()
            {
                if (profile != null)
                {
                    profile.updateConstraints(new Constraints(maxVelocity, maxAcceleration));
                }
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                SlotConfigs config = new SlotConfigs();
                this.getConfigurator().refresh(config);

                p = config.kP;
                i = config.kI;
                d = config.kD;
                v = config.kV;
                a = config.kA;
                s = config.kS;

                following = this.getControlMode().getValue() == com.ctre.phoenix6.signals.ControlModeValue.Follower;

                builder.setActuator(true);
                builder.setSmartDashboardType("ProfiledPIDController");
                builder.publishConstInteger("CanID", canID);

                if (!following) {
                    builder.addDoubleProperty("p", () -> p, (p) -> {this.p = p; applyPID();});
                    builder.addDoubleProperty("i", () -> i, (i) -> {this.i = i; applyPID();});
                    builder.addDoubleProperty("d", () -> d, (d) -> {this.d = d; applyPID();});
                    builder.addDoubleProperty("v", () -> v, (v) -> {this.v = v; applyPID();});
                    builder.addDoubleProperty("a", () -> a, (a) -> {this.a = a; applyPID();});
                    builder.addDoubleProperty("s", () -> s, (s) -> {this.s = s; applyPID();});
                    
                    if (profile != null) {
                        builder.addDoubleProperty("Max Velocity", () -> maxVelocity, (maxVelocity) -> { this.maxVelocity = maxVelocity; applyProfileConstraints(); });
                        builder.addDoubleProperty("Max Acceleration", () -> maxAcceleration, (maxAcceleration) -> { this.maxAcceleration = maxAcceleration; applyProfileConstraints(); });
                        builder.addDoubleProperty("Trapezoid Position", () -> profile.lastState.position, null);
                        builder.addDoubleProperty("Trapezoid Velocity", () -> profile.lastState.velocity, null);
                    }

                    if (slewLimiter != null) {
                        builder.addDoubleProperty("Slew Rate Limit", () -> slewLimiter.rateLimit, (rateLimit) -> slewLimiter.updateRateLimit(rateLimit));
                        builder.addDoubleProperty("Slew Last Value", () -> slewLimiter.lastValue(), null);
                    }

                    if (getSetpoint != null && setSetpoint != null) {
                        builder.addDoubleProperty("goal", getSetpoint, setSetpoint);
                    }
                }

                // General logging (that can't be changed since it's just data)
                builder.addDoubleProperty("Applied Output", () -> this.getDutyCycle().getValueAsDouble(), null);
                builder.addDoubleProperty("Voltage", () -> this.getMotorVoltage().getValueAsDouble(), null);
                builder.addDoubleProperty("Amp usage", () -> this.getSupplyCurrent().getValueAsDouble(), null);
                builder.addDoubleProperty("Position", () -> this.getPosition().getValueAsDouble(), null);
                builder.addDoubleProperty("Velocity", () -> this.getVelocity().getValueAsDouble(), null);
            }
        }
    }

    public static class LoggedTrapezoidProfile extends TrapezoidProfile{
        public Constraints constraints;
        public State lastState = new State();

        private TrapezoidProfile internalTrap;

        public LoggedTrapezoidProfile(TrapezoidProfile.Constraints constraints) {
            super(constraints);
            this.constraints = constraints;
            this.internalTrap = new TrapezoidProfile(this.constraints);
        }

        @Override
        public State calculate(double t, State current, State goal) {
            State state = internalTrap.calculate(t, current, goal);
            this.lastState = state;
            return state;
        }

        public void updateConstraints(TrapezoidProfile.Constraints newConstraints) {
            this.constraints = newConstraints;
            this.internalTrap = new TrapezoidProfile(this.constraints);
        }
    }

    public static class LoggedSlewRateLimiter {
        private double rateLimit;
        private SlewRateLimiter internalLimiter;

        public LoggedSlewRateLimiter(double rateLimit) {
            internalLimiter = new SlewRateLimiter(rateLimit);
            this.rateLimit = rateLimit;
        }

        public double calculate(double input) {
            return internalLimiter.calculate(input);
        }

        public void reset(double value) {
            internalLimiter.reset(value);
        }

        public double lastValue() {
            return internalLimiter.lastValue();
        }

        public void updateRateLimit(double newRateLimit) {
            this.rateLimit = newRateLimit;
            this.internalLimiter = new SlewRateLimiter(newRateLimit);
        }
    }
}
