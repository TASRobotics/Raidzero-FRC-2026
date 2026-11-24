package raidzero.lib.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import raidzero.lib.hardware.CurrentLimits.SupplyLowerLimit;

class CurrentLimitsTest {
    private static final CurrentLimits BASE = CurrentLimits.stator(Amps.of(80)).supply(Amps.of(60));

    @Test
    void supplyNeverLowersUnlessAsked() {
        assertTrue(BASE.supplyLowerLimit().isEmpty());
    }

    @Test
    void keepsTheLowering() {
        SupplyLowerLimit lower = BASE.lowerSupplyTo(Amps.of(40))
                .after(Seconds.of(1))
                .supplyLowerLimit()
                .orElseThrow();
        assertEquals(40, lower.limit().in(Amps), 1e-9);
        assertEquals(1, lower.after().in(Seconds), 1e-9);
    }

    @Test
    void lowerLimitMustBeBelowTheSupplyLimit() {
        assertThrows(IllegalArgumentException.class, () -> BASE.lowerSupplyTo(Amps.of(60))
                .after(Seconds.of(1)));
    }

    @Test
    void delayMustBeInRange() {
        assertThrows(IllegalArgumentException.class, () -> BASE.lowerSupplyTo(Amps.of(40))
                .after(Seconds.of(0)));
        assertThrows(IllegalArgumentException.class, () -> BASE.lowerSupplyTo(Amps.of(40))
                .after(Seconds.of(6)));
    }
}
