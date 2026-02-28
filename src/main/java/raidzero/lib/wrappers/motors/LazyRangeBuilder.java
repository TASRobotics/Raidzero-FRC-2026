package raidzero.lib.wrappers.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class LazyRangeBuilder {
    private int rangeID;
    private CANBus canBus;
    private CANrangeConfiguration configuration;

    /**
     * Constructs a LazyRangeBuilder with the specified device ID and CAN bus.
     *
     * @param rangeID the device id of the CANrange sensor
     * @param canBus  the name of the CAN bus the sensor is on
     */
    public LazyRangeBuilder(int rangeID, CANBus canBus) {
        this.rangeID = rangeID;
        this.canBus = canBus;
        this.configuration = new CANrangeConfiguration();
    }

    /**
     * Configures the field of view (FOV) parameters.
     *
     * @param fovCenterX the horizontal center of the FOV
     * @param fovCenterY the vertical center of the FOV
     * @param fovRangeX  the horizontal range of the FOV
     * @param fovRangeY  the vertical range of the FOV
     * @return this builder
     */
    public LazyRangeBuilder withFovParams(double fovCenterX, double fovCenterY, double fovRangeX, double fovRangeY) {
        configuration.withFovParams(
            new FovParamsConfigs()
                .withFOVCenterX(fovCenterX)
                .withFOVCenterY(fovCenterY)
                .withFOVRangeX(fovRangeX)
                .withFOVRangeY(fovRangeY)
        );
        return this;
    }

    /**
     * Configures the proximity detection parameters.
     *
     * @param proximityThreshold the distance threshold for proximity detection (in meters)
     * @param hysteresis         the hysteresis for proximity detection (in meters)
     * @return this builder
     */
    public LazyRangeBuilder withProximityParams(double proximityThreshold, double hysteresis) {
        configuration.withProximityParams(
            new ProximityParamsConfigs()
                .withProximityThreshold(proximityThreshold)
                .withProximityHysteresis(hysteresis)
        );
        return this;
    }

    /**
     * Configures the proximity detection parameters with only a threshold.
     *
     * @param proximityThreshold the distance threshold for proximity detection (in meters)
     * @return this builder
     */
    public LazyRangeBuilder withProximityParams(double proximityThreshold) {
        configuration.withProximityParams(
            new ProximityParamsConfigs()
                .withProximityThreshold(proximityThreshold)
        );
        return this;
    }

    /**
     * Configures the Time-of-Flight (ToF) parameters.
     *
     * @param updateMode      the update mode for the sensor
     * @param updateFrequency the update frequency in Hz
     * @return this builder
     */
    public LazyRangeBuilder withToFParams(UpdateModeValue updateMode, double updateFrequency) {
        configuration.withToFParams(
            new ToFParamsConfigs()
                .withUpdateMode(updateMode)
                .withUpdateFrequency(updateFrequency)
        );
        return this;
    }

    /**
     * Applies a fully custom configuration, overriding all previous settings.
     *
     * @param configuration the custom CANrangeConfiguration to apply
     * @return this builder
     */
    public LazyRangeBuilder withCustomConfiguration(CANrangeConfiguration configuration) {
        this.configuration = configuration;
        return this;
    }

    /**
     * Returns the current CANrangeConfiguration.
     *
     * @return the CANrangeConfiguration
     */
    public CANrangeConfiguration getConfiguration() {
        return configuration;
    }

    /**
     * Builds and returns the configured LazyRange instance.
     *
     * @return the constructed LazyRange
     */
    public LazyRange build() {
        return new LazyRange(rangeID, canBus, configuration);
    }
}
