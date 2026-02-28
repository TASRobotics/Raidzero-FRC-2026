package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyRange;
import raidzero.lib.wrappers.motors.LazyRangeBuilder;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;
import raidzero.robot.subsystems.shooter.ShooterConstants.Canrange;
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
     * Checks if the feeder has detected an object within the proximity threshold
     *
     * @return true if an object is detected, false otherwise
     */
    boolean ballDetected();

    /**
     * Gets the angular velocity of the feeder motors
     *
     * @return The angular velocity of the feeder motors as an {@link AngularVelocity}
     */
    AngularVelocity getVelocity();

    public class Real implements FeederIO {
        LazyTalon motor;
        LazyRange canrange;
        double lastDetectedTime = Double.NEGATIVE_INFINITY;

        /**
         * Constructs the real instance of the Feeder IO
         */
        public Real() {
            motor = new LazyTalonBuilder(
                Feeder.MOTOR_ID, Superstructure.canDoAttitude,
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

            canrange = new LazyRangeBuilder(Canrange.CAN_ID, Superstructure.canDoAttitude)
                .withFovParams(Canrange.FOV_CENTER_X, Canrange.FOV_CENTER_Y, Canrange.FOV_RANGE_X, Canrange.FOV_RANGE_Y)
                .withProximityParams(Canrange.PROXIMITY_THRESHOLD)
                .withToFParams(Canrange.UPDATE_MODE, Canrange.UPDATE_FREQUENCY)
                .build();
            // Superstructure.orchestra.addInstrument(this.motor.getMotor());
        }

        /**
         * Runs the feeder motors at the given speed
         *
         * @param velocity The speed to run the feeder motors at as an {@link AngularVelocity}
         */
        public void go(AngularVelocity velocity) {
            motor.setPIDVelocityTarget(velocity, 0);
        }

        public boolean ballDetected() {
            if (canrange.isDetected()) {
                lastDetectedTime = Timer.getTimestamp();
            }
            return Timer.getTimestamp() - lastDetectedTime <= Canrange.DETECTION_WINDOW;
        }

        /**
         * Stops the feeder motors
         */
        public void stop() {
            motor.stop();
        }

        /**
         * Gets the angular velocity of the feeder motors
         *
         * @return The angular velocity of the feeder motors as an {@link AngularVelocity}
         */
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