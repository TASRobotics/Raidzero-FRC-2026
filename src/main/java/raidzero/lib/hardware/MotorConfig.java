package raidzero.lib.hardware;

import edu.wpi.first.math.system.plant.DCMotor;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * The physical facts about one motor (and its followers), with no vendor types. The vendor layer
 * turns this into device configuration.
 *
 * <p>Every field is required, so a motor can't be declared without current limits, a direction or
 * its gearing.
 *
 * @param id the leader's address; followers must be on the same bus
 * @param controller the motor controller
 * @param model the motor, which must work with {@code controller}
 * @param direction which way is positive for the mechanism
 * @param neutralMode brake or coast when not driven
 * @param currentLimits applied to the leader and every follower
 * @param gearing motor rotations per mechanism rotation
 * @param followers extra motors on the same shaft; empty if there are none
 */
public record MotorConfig(
        CanId id,
        Controller controller,
        MotorModel model,
        Direction direction,
        NeutralMode neutralMode,
        CurrentLimits currentLimits,
        Gearing gearing,
        List<Follower> followers) {

    public MotorConfig {
        Objects.requireNonNull(id, "id");
        Objects.requireNonNull(controller, "controller");
        Objects.requireNonNull(model, "model");
        Objects.requireNonNull(direction, "direction");
        Objects.requireNonNull(neutralMode, "neutralMode");
        Objects.requireNonNull(currentLimits, "currentLimits");
        Objects.requireNonNull(gearing, "gearing");
        followers = List.copyOf(followers);
        if (!model.worksWith(controller)) {
            throw new IllegalArgumentException(model + " can't be driven by a " + controller);
        }
        Set<Integer> ids = new HashSet<>();
        ids.add(id.id());
        for (Follower follower : followers) {
            if (!ids.add(follower.id())) {
                throw new IllegalArgumentException("CAN ID " + follower.id() + " is used twice");
            }
        }
    }

    /** The leader plus its followers. */
    public int motorCount() {
        return 1 + followers.size();
    }

    /** All the motors together, for simulation. Followers are assumed to be the same model. */
    public DCMotor dcMotor() {
        return model.dcMotor(motorCount());
    }

    /**
     * Motor controllers. When adding one, the compiler shows every switch that must handle it, as
     * long as those switches have no {@code default}.
     */
    public enum Controller {
        TALON_FX,
        TALON_FXS
    }

    /** Motors, and which controllers can drive them. */
    public enum MotorModel {
        KRAKEN_X60(Controller.TALON_FX),
        KRAKEN_X44(Controller.TALON_FX),
        FALCON_500(Controller.TALON_FX),
        MINION(Controller.TALON_FXS),
        NEO(Controller.TALON_FXS),
        NEO_550(Controller.TALON_FXS),
        VORTEX(Controller.TALON_FXS);

        private final Controller controller;

        MotorModel(Controller controller) {
            this.controller = controller;
        }

        /**
         * Whether {@code controller} can drive this motor. One controller per motor today; widen
         * this when a motor gains a second option (e.g. a NEO on a SPARK).
         */
        public boolean worksWith(Controller controller) {
            return this.controller == controller;
        }

        /** The physics model for {@code count} of these motors geared together. */
        public DCMotor dcMotor(int count) {
            return switch (this) {
                case KRAKEN_X60 -> DCMotor.getKrakenX60(count);
                case KRAKEN_X44 -> DCMotor.getKrakenX44(count);
                case FALCON_500 -> DCMotor.getFalcon500(count);
                case MINION -> DCMotor.getMinion(count);
                case NEO -> DCMotor.getNEO(count);
                case NEO_550 -> DCMotor.getNeo550(count);
                case VORTEX -> DCMotor.getNeoVortex(count);
            };
        }
    }

    /** Which way the mechanism moves for a positive output, looking at the motor's face. */
    public enum Direction {
        CLOCKWISE_POSITIVE,
        COUNTERCLOCKWISE_POSITIVE
    }

    /** What the motor does when not driven. */
    public enum NeutralMode {
        BRAKE,
        COAST
    }

    /**
     * A motor that copies the leader's output. Use followers only for motors on the same rigid
     * output as the leader; sides that can move relative to each other each need their own leader.
     *
     * @param id CAN ID, on the leader's bus
     * @param opposed true if it's mounted mirrored and must spin the opposite way
     */
    public record Follower(int id, boolean opposed) {
        public Follower {
            new CanId(id, CanBus.RIO); // range check only
        }
    }

    /**
     * Motor rotations per mechanism rotation.
     *
     * @param reduction greater than 1 for a reduction, which is almost always the case
     */
    public record Gearing(double reduction) {
        public Gearing {
            if (!(reduction > 0)) {
                throw new IllegalArgumentException("Gear reduction must be positive, got " + reduction);
            }
        }

        /**
         * Builds the reduction from tooth counts, one stage at a time.
         *
         * @param teeth pairs of (driven, driving), e.g. {@code stages(42, 10, 42, 14)} for 42:10 then
         *     42:14
         * @return the combined reduction
         */
        public static Gearing stages(double... teeth) {
            if (teeth.length == 0 || teeth.length % 2 != 0) {
                throw new IllegalArgumentException("stages() takes pairs of (driven, driving) teeth");
            }
            double reduction = 1;
            for (int i = 0; i < teeth.length; i += 2) {
                if (!(teeth[i] > 0) || !(teeth[i + 1] > 0)) {
                    throw new IllegalArgumentException("Tooth counts must be positive");
                }
                reduction *= teeth[i] / teeth[i + 1];
            }
            return new Gearing(reduction);
        }
    }
}
