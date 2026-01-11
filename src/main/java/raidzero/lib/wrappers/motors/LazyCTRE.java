package raidzero.lib.wrappers.motors;

import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * I call it an interface for wrapping CTRE motors, rhymes with Grug
 *
 * @param <T> is the motor ojbect to use
 * @param <S> is the sensor type value to use for CANcoders
 */
public interface LazyCTRE<T, S> {
    /**
     * Configures a follower motor
     *
     * @param followerID the device ID of the follower motor
     * @param isInverted if true, the follower output is inverted with respect to the master
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withFollower(int followerID, boolean isInverted);

    /**
     * Configures a {@link CANcoder} remote feedback sensor
     *
     * @param CANCoderID the device ID of the CANcoder
     * @param sensorType the type of feedback the CANcoder will supply
     * @param magnetOffset the digital magnet offset applied to the sensor
     * @param sensorDirection the sensor direciton relative to the mechanism
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withCANCoder(int CANCoderID, S sensorType, double magnetOffset, SensorDirectionValue sensorDirection);

    /**
     * Configures a {@link CANcoder} remote feedback sensor with a specified rotor to sensor ratio and discontinuity point
     *
     * @param CANCoderID the device ID of the CANcoder
     * @param sensorType the type of feedback the CANcoder will supply
     * @param magnetOffset the digital magnet offset applied to the sensor
     * @param sensorDirection the sensor direciton relative to the mechanism
     * @param discontinuityPoint the point of discontinuity for the sensor
     * @param rotorToSensorRatio the gear ratio of the sensor to the motor
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withCANCoder(int CANCoderID, S sensorType, double magnetOffset, SensorDirectionValue sensorDirection, double discontinuityPoint, double rotorToSensorRatio);

    /**
     * Configures the motion magic parameters for this motor's motion prifiling
     *
     * @param p the proportional gain
     * @param i the integral gain
     * @param d the derivative gain
     * @param s the static friction feed forward (voltage reqiured to overcome static friction)
     * @param g the gravity feed forward (voltage required to overcome gravity)
     * @param v the velocity feed forward (voltage required to maintain a 1 rotation/s angular velocity in mechanism rotations)
     * @param a the velocity feed forward (voltage required to maintain a 1 rotation/s^2 angular acceleration in mechanism rotations)
     * @param gravityType what type of gravity feed forward to use
     * @param cruiseVelocity the maximum velocity of the mechanism in rotations/s
     * @param maxAcceleration the maximum acceleration of the mechanism in rotations/s^2
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withMotionMagicConfiguration(double p, double i, double d, double s, double g, double v, double a, double expoV, double expoA, GravityTypeValue gravityType, double cruiseVelocity, double maxAcceleration);

    /**
     * Configures the forward and reverse hardware limit switches
     *
     * @param forwardlimitenable if true, enables the forward limit switch
     * @param forwardLimitAutosetPositionEnable if true, the encoder position will be set to the forward limit autoset position value when the limit is triggered
     * @param forwardLimitAutosetPositionValue the value to set the encoder position to when the limit is triggered
     * @param reverseLimitEnable if true, enables the reverse limit switch
     * @param reverseLimitAutosetPositionEnable if true, the encoder position will be set to the reverse limit autoset position value when the limit is triggered
     * @param reverseLimitAutosetPositionValue if true, enables the forward limit switch
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withLimitSwitch(boolean forwardLimitEnable, boolean forwardLimitAutosetPositionEnable, double forwardLimitAutosetPositionValue, boolean reverseLimitEnable, boolean reverseLimitAutosetPositionEnable, double reverseLimitAutosetPositionValue);

    /**
     * Configures the forward and reverse software limit switches
     *
     * @param forwardSoftLimitEnable if true, enables the forward software limit switch
     * @param forwardSoftLimit the encoder value of the forward software limit in mechanism rotations
     * @param reverseSoftLimitEnable if true, enables the reverse software limit switch
     * @param reverseSoftLimit the encoder value of the reverse software limit in mechanism rotations
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withSoftLimits(boolean forwardSoftLimitEnable, double forwardSoftLimit, boolean reverseSoftLimitEnable, double reverseSoftLimit);

    /**
     * Applies a custom configuration to this motor

     * @param configuration the configuration object specific to the motor type
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> withCustomConfiguration(ParentConfiguration configuration);

    /**
     * Gets the motor configuration
     *
     * @return the motor's configuration
     */
    public ParentConfiguration getConfiguration();

    /**
     * Applies the motor configuration. Use this method at the end of the configuration method chain
     * @return this motor instance for method chaining
     */
    public LazyCTRE<T, S> build();

    /**
     * Gets the CANcoder remote sensor
     *
     * @return the {@link CANcoder} object
     */
    public CANcoder getCanCoder();
}
