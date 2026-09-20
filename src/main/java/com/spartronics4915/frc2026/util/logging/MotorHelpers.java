// Utility class for logging Krakens to SmartDashboard

package com.spartronics4915.frc2026.util.logging;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.Arrays;
import java.util.LinkedHashSet;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;

public class MotorHelpers {
    public class CTRE {
        public static class LoggedTalonFX extends TalonFX {
            // Shared Phoenix handles: subsystem control and dashboard callbacks read the same sample.
            private final BaseStatusSignal appliedOutputSignal;
            private final BaseStatusSignal voltageSignal;
            private final BaseStatusSignal supplyCurrentSignal;
            private final BaseStatusSignal positionSignal;
            private final BaseStatusSignal velocitySignal;
            private final BaseStatusSignal[] dashboardSignals;

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
                appliedOutputSignal = getDutyCycle(false);
                voltageSignal = getMotorVoltage(false);
                supplyCurrentSignal = getSupplyCurrent(false);
                positionSignal = getPosition(false);
                velocitySignal = getVelocity(false);
                dashboardSignals = new BaseStatusSignal[] {appliedOutputSignal, voltageSignal, supplyCurrentSignal, positionSignal, velocitySignal};
            }

            /**
             * Builds a reusable refresh group containing dashboard and subsystem signals.
             * Call once during subsystem construction, then refreshAll(group) at the beginning
             * of periodic(). Additional signals must be on this motor's CAN bus. Shared handles
             * are included only once; dashboard callbacks never refresh hardware themselves.
             */
            public BaseStatusSignal[] createTelemetrySignalGroup(BaseStatusSignal... additionalSignals) {
                var signals = new LinkedHashSet<BaseStatusSignal>(Arrays.asList(dashboardSignals));
                signals.addAll(Arrays.asList(additionalSignals));
                return signals.toArray(BaseStatusSignal[]::new);
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
                        .withKS(s));
            }

            private void applyProfileConstraints() {
                if (profile != null) {
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

                following = this.getControlMode().getValue() == ControlModeValue.Follower;
                // Seed the dashboard once after configuration; periodic owns subsequent refreshes.
                BaseStatusSignal.refreshAll(dashboardSignals);

                builder.setActuator(true);
                builder.setSmartDashboardType("ProfiledPIDController");
                builder.publishConstInteger("CanID", canID);

                if (!following) {
                    builder.addDoubleProperty("p", () -> p, (p) -> {
                        this.p = p;
                        applyPID();
                    });
                    builder.addDoubleProperty("i", () -> i, (i) -> {
                        this.i = i;
                        applyPID();
                    });
                    builder.addDoubleProperty("d", () -> d, (d) -> {
                        this.d = d;
                        applyPID();
                    });
                    builder.addDoubleProperty("v", () -> v, (v) -> {
                        this.v = v;
                        applyPID();
                    });
                    builder.addDoubleProperty("a", () -> a, (a) -> {
                        this.a = a;
                        applyPID();
                    });
                    builder.addDoubleProperty("s", () -> s, (s) -> {
                        this.s = s;
                        applyPID();
                    });

                    if (profile != null) {
                        builder.addDoubleProperty("Max Velocity", () -> maxVelocity, (maxVelocity) -> {
                            this.maxVelocity = maxVelocity;
                            applyProfileConstraints();
                        });
                        builder.addDoubleProperty("Max Acceleration", () -> maxAcceleration, (maxAcceleration) -> {
                            this.maxAcceleration = maxAcceleration;
                            applyProfileConstraints();
                        });
                        builder.addDoubleProperty("Trapezoid Position", () -> profile.lastState.position, null);
                        builder.addDoubleProperty("Trapezoid Velocity", () -> profile.lastState.velocity, null);
                    }

                    if (slewLimiter != null) {
                        builder.addDoubleProperty("Slew Rate Limit", () -> slewLimiter.rateLimit,
                                (rateLimit) -> slewLimiter.updateRateLimit(rateLimit));
                        builder.addDoubleProperty("Slew Last Value", () -> slewLimiter.lastValue(), null);
                    }

                    if (getSetpoint != null && setSetpoint != null) {
                        builder.addDoubleProperty("goal", getSetpoint, setSetpoint);
                    }
                }

                // General logging (that can't be changed since it's just data)
                builder.addDoubleProperty("Applied Output", appliedOutputSignal::getValueAsDouble, null);
                builder.addDoubleProperty("Voltage", voltageSignal::getValueAsDouble, null);
                builder.addDoubleProperty("Amp usage", supplyCurrentSignal::getValueAsDouble, null);
                builder.addDoubleProperty("Position", positionSignal::getValueAsDouble, null);
                builder.addDoubleProperty("Velocity", velocitySignal::getValueAsDouble, null);
            }
        }
    }

    public static class LoggedTrapezoidProfile extends TrapezoidProfile {
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
