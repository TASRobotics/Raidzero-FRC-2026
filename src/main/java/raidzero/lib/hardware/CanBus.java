package raidzero.lib.hardware;

import java.util.Objects;

/**
 * A CAN bus, with no vendor types. The vendor layer turns it into the vendor's bus object.
 *
 * @param name {@code "rio"} for the roboRIO's bus, otherwise the CANivore's name
 */
public record CanBus(String name) {
    /** The roboRIO's built-in CAN bus. */
    public static final CanBus RIO = new CanBus("rio");

    public CanBus {
        Objects.requireNonNull(name, "name");
        if (name.isBlank()) {
            throw new IllegalArgumentException("bus name must not be blank; use CanBus.RIO for the roboRIO");
        }
    }

    /** Whether this is the roboRIO's built-in bus. */
    public boolean isRio() {
        return equals(RIO);
    }
}
