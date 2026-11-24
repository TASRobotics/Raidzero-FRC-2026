package raidzero.lib.hardware;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Objects;

/**
 * How a motor controller holds a position with its onboard closed loop. Shared by every
 * positional archetype (Pivot today; Elevator and Turret later). All values are in mechanism
 * rotations, so gains don't change when the gearing does.
 *
 * @param gains feedback on position error
 * @param feedforward model-based output
 * @param gravity how gravity loads the mechanism; {@code kG} must be 0 for {@link Gravity#NONE}
 * @param profile how goals are approached; there is no default
 * @param softLimits the allowed travel, enforced on the device and by the archetype
 */
public record PositionControl(
        Gains gains, Feedforward feedforward, Gravity gravity, Profile profile, SoftLimits softLimits) {

    public PositionControl {
        Objects.requireNonNull(gains, "gains");
        Objects.requireNonNull(feedforward, "feedforward");
        Objects.requireNonNull(gravity, "gravity");
        Objects.requireNonNull(profile, "profile");
        Objects.requireNonNull(softLimits, "softLimits");
        if (gravity == Gravity.NONE && feedforward.kG() != 0) {
            throw new IllegalArgumentException("kG must be 0 when gravity is NONE");
        }
    }

    /**
     * Feedback gains, in volts per rotation of error (kP), per rotation-second (kI) and per
     * rotation/second (kD).
     */
    public record Gains(double kP, double kI, double kD) {
        public Gains {
            if (kP < 0 || kI < 0 || kD < 0) {
                throw new IllegalArgumentException("Gains must not be negative");
            }
        }

        /** Proportional-only gains, the usual starting point. */
        public static Gains p(double kP) {
            return new Gains(kP, 0, 0);
        }
    }

    /**
     * Feedforward, in volts (kS, kG), volts per rotation/second (kV) and volts per
     * rotation/second² (kA). kG may be negative if gravity pulls toward positive.
     */
    public record Feedforward(double kS, double kG, double kV, double kA) {
        public Feedforward {
            if (kS < 0 || kV < 0 || kA < 0) {
                throw new IllegalArgumentException("kS, kV and kA must not be negative");
            }
        }
    }

    /** How gravity loads the mechanism. */
    public enum Gravity {
        /** No gravity load, e.g. a turret. */
        NONE,
        /** Load varies with the cosine of position; position 0 must be horizontal. */
        COSINE
    }

    /** How the controller moves toward a goal. One of the nested records. */
    public sealed interface Profile {
        /** Constant acceleration up to a cruise velocity. */
        record Trapezoid(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) implements Profile {
            public Trapezoid {
                if (!(cruiseVelocity.in(RotationsPerSecond) > 0)
                        || !(acceleration.in(RotationsPerSecondPerSecond) > 0)) {
                    throw new IllegalArgumentException("Cruise velocity and acceleration must be positive");
                }
            }
        }

        /**
         * A profile shaped by the mechanism's own kV and kA, which slows smoothly near the goal.
         *
         * @param kV volts per rotation/second; lower is faster
         * @param kA volts per rotation/second²; lower is faster
         * @param maxVelocity speed cap
         */
        record Exponential(double kV, double kA, AngularVelocity maxVelocity) implements Profile {
            public Exponential {
                if (!(kV > 0) || !(kA > 0) || !(maxVelocity.in(RotationsPerSecond) > 0)) {
                    throw new IllegalArgumentException("Exponential kV, kA and maxVelocity must be positive");
                }
            }
        }
    }

    /** Allowed travel, reverse below forward. */
    public record SoftLimits(Angle reverse, Angle forward) {
        public SoftLimits {
            Objects.requireNonNull(reverse, "reverse");
            Objects.requireNonNull(forward, "forward");
            if (!(reverse.in(Rotations) < forward.in(Rotations))) {
                throw new IllegalArgumentException("Reverse soft limit must be below forward");
            }
        }

        /** Whether {@code angle} is within the limits, inclusive. */
        public boolean contains(Angle angle) {
            double r = angle.in(Rotations);
            return r >= reverse.in(Rotations) && r <= forward.in(Rotations);
        }

        /** {@code angle} moved to the nearest limit if it's outside them. */
        public Angle clamp(Angle angle) {
            double r = angle.in(Rotations);
            return Rotations.of(Math.max(reverse.in(Rotations), Math.min(forward.in(Rotations), r)));
        }

        /** Forward minus reverse. */
        public Angle span() {
            return forward.minus(reverse);
        }
    }
}
