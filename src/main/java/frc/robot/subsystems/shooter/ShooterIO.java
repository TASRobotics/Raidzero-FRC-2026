package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;

import org.wpilib.units.measure.AngularVelocity;
import frc.lib.SubsystemIO;
import frc.lib.wrappers.motors.LazyTalon;
import frc.lib.wrappers.motors.LazyTalonBuilder;
import frc.robot.Superstructure;
import frc.robot.subsystems.shooter.ShooterConstants.Flywheel;

public interface ShooterIO extends SubsystemIO {

    /**
     * Sets the shooter velocity to the specified speed
     *
     * @param velocity the {@link AngularVelocity}
     */
    void setVelocity(AngularVelocity velocity);

    /**
     * Gets the flywheel's current velocity
     *
     * @return the {@link Velocity}
     */
    AngularVelocity getVelocity();

    /**
     * Sets the shooter to a speed as a percentage
     *
     * @param speedPercent the percent [0, 1]
     */
    void setPercent(double speedPercent);

    /**
     * Stops the flywheel
     */
    void stop();

    public class Real implements ShooterIO {
        LazyTalon flywheel;

        Real() {
            flywheel = new LazyTalonBuilder(
                Flywheel.MOTOR_ID, Superstructure.canDoAttitude,
                Flywheel.SENSOR_TO_MECHANISM_RATIO, Flywheel.INVERTED,
                Flywheel.STATOR_CURRENT_LIMIT.in(Amps),
                Flywheel.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Flywheel.P, Flywheel.I, Flywheel.D,
                    Flywheel.S, Flywheel.G, Flywheel.V, Flywheel.A,
                    Flywheel.FF_GRAVITY_TYPE, Flywheel.FF_SIGN, Flywheel.VELOCITY_SLOT
                )
                .withMotionMagicConfiguration(
                    0, 0, Flywheel.CRUISE_VELOCITY,
                    Flywheel.ACCELERATION
                )
                .withFollower(Flywheel.FOLLOW_ID, Superstructure.canDoAttitude, Flywheel.FOLLOWER_INVERTED)
                .build();

            flywheel.setCoast();
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            this.flywheel.setVelocityTorqueCurrentTarget(velocity, Flywheel.TORQUE_FF_A, 0);
        }

        @Override
        public void setPercent(double speedPercent) {
            flywheel.setDutyCycle(speedPercent);
        }

        @Override
        public void stop() {
            flywheel.stop();
        }

        @Override
        public AngularVelocity getVelocity() {
            return flywheel.getVelocity();
        }

        @Override
        public void updateTelemetry() {
            flywheel.updateTelemetry(Flywheel.TELEMETRY);
        }

    }

    public class Sim extends Real {}

}