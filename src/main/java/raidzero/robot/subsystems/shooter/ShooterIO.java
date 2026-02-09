package raidzero.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Amps;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.subsystems.shooter.ShooterConstants.Flywheel;

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
        LazyTalon flywheel = new LazyTalonBuilder(
                Flywheel.MOTOR_ID, Flywheel.CAN_BUS_NAME,
                Flywheel.SENSOR_TO_MECHANISM_RATIO, Flywheel.INVERTED,
                Flywheel.STATOR_CURRENT_LIMIT.in(Amps),
                Flywheel.SUPPLY_CURRENT_LIMIT.in(Amps))
                .withPIDFConfiguration(
                        Flywheel.P, Flywheel.I, Flywheel.D,
                        Flywheel.S, Flywheel.G, Flywheel.V, Flywheel.A,
                        Flywheel.FF_GRAVITY_TYPE, Flywheel.FF_SIGN, Flywheel.VELOCITY_SLOT)
                .withMotionMagicConfiguration(0, 0, Flywheel.CRUISE_VELOCITY,
                        Flywheel.ACCELERATION)
                .withFollower(Flywheel.FOLLOW_ID, Flywheel.CAN_BUS_NAME, Flywheel.FOLLOWER_INVERTED)
                .build();

        Real() {
            flywheel.setCoast();
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            this.flywheel.setPIDVelocityTarget(velocity, 0);
        }

        @Override
        public void setPercent(double speedPercent) {
            flywheel.setDutyCycle(speedPercent);
        }

        @Override
        public void stop() {
            flywheel.stop();
        }

        public AngularVelocity getVelocity() {
            return flywheel.getVelocity();
        }

        @Override
        public void updateTelemetry() {
            flywheel.updateTelemetry(Flywheel.TELEMETRY);
        }

    }

    public class Sim extends Real {
    }

}