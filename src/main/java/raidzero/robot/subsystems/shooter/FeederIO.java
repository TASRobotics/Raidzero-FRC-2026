package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.subsystems.shooter.ShooterConstants.Feeder;

public interface FeederIO extends SubsystemIO {
    /**
     * Runs the feeder motors at the given speed
     *
     * @param speed The speed to run the feeder motors at as an {@link AngularVelocity}
     */
    void go(AngularVelocity speed);

    /**
     * Stops the feeder motors
     */
    void stop();

    /**
     * Gets the angular velocity of the feeder motors
     *
     * @return The angular velocity of the feeder motors as an {@link AngularVelocity}
     */
    AngularVelocity getVelocity();

    public class Real implements FeederIO {
        LazyTalon motor;

        /**
         * Constructs the real instance of the Feeder IO
         */
        public Real() {
            motor = new LazyTalonBuilder(
                Feeder.MOTOR_ID, Feeder.CAN_BUS_NAME,
                Feeder.SENSOR_TO_MECHANISM_RATIO, Feeder.INVERTED,
                Feeder.STATOR_CURRENT_LIMIT.in(Amps), Feeder.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Feeder.P, Feeder.I, Feeder.D,
                    Feeder.S, Feeder.G, Feeder.V, Feeder.A,
                    Feeder.FF_GRAVITY_TYPE, Feeder.FF_SIGN,
                    0
                )
                .withMotionMagicConfiguration(
                    0, 0,
                    Feeder.CRUISE_VELOCITY,
                    Feeder.ACCELERATION
                )
                .build();

            // Superstructure.orchestra.addInstrument(this.motor.getMotor());
        }

        @Override
        public void go(AngularVelocity velocity) {
            motor.setPIDVelocityTarget(velocity, 0);
        }

        @Override
        public void stop() {
            motor.stop();
        }

        @Override
        public AngularVelocity getVelocity() {
            return motor.getVelocity();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(Feeder.TELEMETRY);
        }

    }

    public class Sim extends Real {}
}
