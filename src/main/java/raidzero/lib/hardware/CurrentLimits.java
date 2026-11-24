package raidzero.lib.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import java.util.Objects;
import java.util.Optional;

/**
 * Stator and supply current limits. Every motor must have both: stator protects the motor and
 * mechanism, supply protects the battery and breakers.
 *
 * <p>Built as {@code CurrentLimits.stator(Amps.of(40)).supply(Amps.of(50))}, so the values are named
 * where they're written and can't be swapped. Optionally, the supply limit can drop to a lower
 * limit after the motor has been held at it for a while, which stops a stalled mechanism from
 * draining the battery or tripping a breaker:
 *
 * <pre>{@code
 * CurrentLimits.stator(Amps.of(80)).supply(Amps.of(60)).lowerSupplyTo(Amps.of(40)).after(Seconds.of(1))
 * }</pre>
 *
 * <p>Without {@code lowerSupplyTo}, the supply limit never lowers.
 */
public final class CurrentLimits {
    /** The longest wait before lowering that motor controllers support (Phoenix 6's maximum). */
    public static final Time MAX_LOWER_TIME = Seconds.of(5);

    private final Current stator;
    private final Current supply;
    private final SupplyLowerLimit supplyLowerLimit;

    private CurrentLimits(Current stator, Current supply, SupplyLowerLimit supplyLowerLimit) {
        Objects.requireNonNull(stator, "stator");
        Objects.requireNonNull(supply, "supply");
        if (stator.in(Amps) <= 0 || supply.in(Amps) <= 0) {
            throw new IllegalArgumentException("Current limits must be positive");
        }
        if (supplyLowerLimit != null && supplyLowerLimit.limit().in(Amps) >= supply.in(Amps)) {
            throw new IllegalArgumentException("The lower supply limit must be below the supply limit");
        }
        this.stator = stator;
        this.supply = supply;
        this.supplyLowerLimit = supplyLowerLimit;
    }

    /**
     * Starts the limits with the stator limit; continue with {@link StatorLimit#supply}.
     *
     * @param stator the stator current limit
     * @return the first part of the limits
     */
    public static StatorLimit stator(Current stator) {
        return new StatorLimit(stator);
    }

    /** The stator current limit. */
    public Current statorLimit() {
        return stator;
    }

    /** The supply current limit. */
    public Current supplyLimit() {
        return supply;
    }

    /** What the supply limit lowers to and when, or empty if it never lowers. */
    public Optional<SupplyLowerLimit> supplyLowerLimit() {
        return Optional.ofNullable(supplyLowerLimit);
    }

    /**
     * Lowers the supply limit after the motor has been held at it; finish with {@link
     * SupplyLowering#after}.
     *
     * @param limit the lower supply limit, below the supply limit
     * @return the lowering, waiting for its delay
     */
    public SupplyLowering lowerSupplyTo(Current limit) {
        return new SupplyLowering(this, limit);
    }

    /**
     * The stator part of {@link CurrentLimits}, waiting for the supply limit.
     *
     * @param stator the stator current limit
     */
    public record StatorLimit(Current stator) {
        /**
         * Adds the supply limit.
         *
         * @param supply the supply current limit
         * @return both limits, with no lowering
         */
        public CurrentLimits supply(Current supply) {
            return new CurrentLimits(stator, supply, null);
        }
    }

    /**
     * A lower supply limit, waiting for how long the motor must be held at the supply limit first.
     *
     * @param base the limits being lowered
     * @param limit the lower supply limit
     */
    public record SupplyLowering(CurrentLimits base, Current limit) {
        /**
         * Finishes the limits.
         *
         * @param time how long the motor is held at the supply limit before it lowers
         * @return the limits with the lowering
         */
        public CurrentLimits after(Time time) {
            return new CurrentLimits(base.stator, base.supply, new SupplyLowerLimit(limit, time));
        }
    }

    /**
     * The supply limit drops to {@code limit} once the motor has been held at the supply limit for
     * {@code after}.
     *
     * @param limit the lower supply limit
     * @param after how long the motor is held at the supply limit first, up to {@link
     *     #MAX_LOWER_TIME}
     */
    public record SupplyLowerLimit(Current limit, Time after) {
        public SupplyLowerLimit {
            Objects.requireNonNull(limit, "limit");
            Objects.requireNonNull(after, "after");
            if (limit.in(Amps) <= 0) {
                throw new IllegalArgumentException("The lower supply limit must be positive");
            }
            if (after.in(Seconds) <= 0 || after.gt(MAX_LOWER_TIME)) {
                throw new IllegalArgumentException(
                        "The lowering delay must be over 0 s and at most " + MAX_LOWER_TIME.in(Seconds) + " s");
            }
        }
    }
}
