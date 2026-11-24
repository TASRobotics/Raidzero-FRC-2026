package raidzero.lib.hardware;

import java.util.Objects;

/**
 * Where a device lives on CAN.
 *
 * @param id device ID, 0 to 62
 * @param bus the bus it's wired to
 */
public record CanId(int id, CanBus bus) {
    public CanId {
        Objects.requireNonNull(bus, "bus");
        if (id < 0 || id > 62) {
            throw new IllegalArgumentException("CAN ID must be 0-62, got " + id);
        }
    }
}
