package raidzero.lib.hardware;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;
import raidzero.lib.hardware.PositionControl.Feedforward;
import raidzero.lib.hardware.PositionControl.Gains;
import raidzero.lib.hardware.PositionControl.Gravity;
import raidzero.lib.hardware.PositionControl.Profile;
import raidzero.lib.hardware.PositionControl.SoftLimits;

class PositionControlTest {
    @Test
    void rejectsGravityGainWithoutGravity() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new PositionControl(
                        Gains.p(1),
                        new Feedforward(0, 0.2, 0, 0),
                        Gravity.NONE,
                        new Profile.Exponential(1, 1, RotationsPerSecond.of(1)),
                        new SoftLimits(Rotations.of(0), Rotations.of(1))));
    }

    @Test
    void softLimitsMustBeOrdered() {
        assertThrows(IllegalArgumentException.class, () -> new SoftLimits(Rotations.of(1), Rotations.of(1)));
    }

    @Test
    void clampsToTheNearestLimit() {
        SoftLimits limits = new SoftLimits(Rotations.of(0.1), Rotations.of(0.5));
        assertEquals(0.5, limits.clamp(Rotations.of(2)).in(Rotations), 1e-9);
        assertEquals(0.1, limits.clamp(Rotations.of(-2)).in(Rotations), 1e-9);
        assertEquals(0.3, limits.clamp(Rotations.of(0.3)).in(Rotations), 1e-9);
    }

    @Test
    void rejectsNegativeGains() {
        assertThrows(IllegalArgumentException.class, () -> Gains.p(-1));
    }
}
