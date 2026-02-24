package raidzero.lib.wrappers.motors;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface LazyCTRE {
    /**
     * Sets the target position for the motor using Motion Magic Position Voltage
     * control
     * request
     * This request uses the PID gains, feedforward, and motion magic acceleration,
     * velocity, and jerk
     * 
     * @param setpoint the mechanism-relative setpoint
     */
    void setMMPositionTarget(Angle setpoint, int slot);

    /**
     * Sets the target position for the motor using Motion Magic Expo Voltage
     * control
     * This request uses the PID gains, feedforward, and motion expo kv and ka,
     * 
     * @param setpoint the mechanism-relative setpoint
     */
    void setMMExpoTarget(Angle setpoint, int slot);

    /**
     * Sets the target velocity for the motor using Motion Magic Velocity Voltage
     * control
     * This request uses the PID gains, feedforward, and motion magic acceleration
     * and jerk
     * 
     * @param velocity the mechanism-relative velocity setpoint
     */
    void setMMVelocityTarget(AngularVelocity velocity, int slot);

    /**
     * Sets the target position for the motor using Position Voltage control
     * This request uses the PID gains and feedforward
     * 
     * @param setpoint the mechanism-relative setpoint
     */
    void setPIDPositionTarget(Angle setpoint, int slot);

    /**
     * Sets the target velocity for the motor using Velocity Voltage control
     * This request uses the PID gains and feedforward
     * 
     * @param velocity the mechanism-relative velocity setpoint
     */
    void setPIDVelocityTarget(AngularVelocity velocity, int slot);

    /**
     * Sets the motor output to a percentage of maximum output, where 1.0 represents
     * full forward output and -1.0 represents full reverse output
     * 
     * @param percent the output percentage [-1, 1]
     */
    void setDutyCycle(double percent);

    /**
     * Sets the motor's {@link NeutralModeValue} to Coast, allowing the motor to
     * spin freely when no power is applied
     */
    void setCoast();

    /**
     * Sets the motor's {@link NeutralModeValue} to Brake, causing the motor to
     * resist motion when no power is applied
     */
    void setBrake();

    /**
     * Enalbes FOC, increasing the motor's peak power by 15%
     */
    void enableFOC();

    /**
     * Disables FOC, returning the motor's peak power to normal
     */
    void disableFOC();

    /**
     * Stops the motor
     */
    void stop();

    /**
     * Gets the current motor feedback position in mechanism-relative units
     * 
     * @return the current mechanism position
     */
    Angle getPosition();

    /**
     * Gets the current motor feedback velocity in mechanism-relative units
     * 
     * @return the current mechanism velocity
     */
    AngularVelocity getVelocity();

    /**
     * Gets the current motor feedback acceleration in mechanism-relative units
     * 
     * @return the current mechanism acceleration
     */
    AngularAcceleration getAcceleration();

    /**
     * Updates the telemetry data for the motor
     * @param telemetry the telemetry object to update
     */
    public void updateTelemetry(MotorTelemetry telemetry);

    @AutoLog
    public static class MotorTelemetry {
        public Angle position;
        public AngularVelocity velocity;
        public AngularAcceleration acceleration;
        public Current statorCurrent;
        public Current supplyCurrent;
    }
}