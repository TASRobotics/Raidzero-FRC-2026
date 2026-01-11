package raidzero.lib.wrappers.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface LazyMotor<T> {
    /**
     * Updates motor state data
     */
    public void updateTelemetry(MotorTelemetry telemetry);

    /**
     * Moves the motor to the supplied position setpoint using a motion magic exponential voltage profile
     *
     * @param setpoint the target position setpoint
     */
    public void moveTo(Angle setpoint);

    /**
     * Moves the motor at the specified velocity using a motion magic velocity voltage profile
     *
     * @param velocity the target velocity for motion magic control
     */
    public void moveWithVelocity(AngularVelocity velocity);

    /**
     * Sets the motor to the target speed

     * @param speed the speed in percent of max speed [0, 1]
     */
    public void set(double speed);

    /**
     * Stops the motor
     */
    public void stop();

    /**
     * Sets the idle/neutral mode of the motor to brake
     */
    public void setBrake();

    /**
     * Sets the idle/neutral mode of the motor to coast
     */
    public void setCoast();

    /**
     * Gets the current feedback position of the motor
     *
     * @return the feedback position as an {@link Angle} object
     */
    public Angle getFeedbackPosition();

    /**
     * Gets the current feedvack velocity of the motor
     *
     * @return the feedback velocity as an {@link AngularVelocity} object
     */
    public AngularVelocity getFeedbackVelocity();

    /**
     * Gets the current feedback acceleration of the motor
     * @return the feedback acceleration as an {@link AngularAcceleration} object
     */
    public AngularAcceleration getFeedbackAcceleration();

    /**
     * Gets the motor object
     *
     * @return the motor
     */
    public T getMotor();

    /**
     * Gets the follower motor
     *
     * @return the follower motor
     */
    public T getFollower();

    @AutoLog
    public static class MotorTelemetry {
        public Angle position;
        public AngularVelocity velocity;
        public AngularAcceleration acceleration;
        public Current statorCurrent;
        public Current supplyCurrent;
    }

}
