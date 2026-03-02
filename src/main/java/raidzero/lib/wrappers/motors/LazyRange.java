package raidzero.lib.wrappers.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;

public class LazyRange {
    private CANrange canRange;

    /**
     * Constructs a LazyRange instance with the specified configuration.
     *
     * @param rangeID       the device id of the CANrange sensor
     * @param canBus        the name of the CAN bus
     * @param configuration the CANrangeConfiguration to apply
     */
    public LazyRange(int rangeID, CANBus canBus, CANrangeConfiguration configuration) {
        canRange = new CANrange(rangeID, canBus);
        canRange.getConfigurator().apply(configuration);
    }

    /**
     * Returns the measured distance from the CANrange sensor.
     *
     * @return the distance as a {@link Distance}
     */
    public Distance getDistance() {
        return canRange.getDistance().getValue();
    }

    /**
     * Returns whether the CANrange sensor detects an object within the proximity
     * threshold.
     *
     * @return true if an object is detected, false otherwise
     */
    public boolean isDetected() {
        return canRange.getIsDetected().getValue();
    }

    /**
     * Returns the underlying CANrange hardware object.
     *
     * @return the {@link CANrange} instance
     */
    public CANrange getRange() {
        return canRange;
    }

    /**
     * Updates the telemetry for the CANrange sensor.
     */
    public void updateTelemetry() {}
}