package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.subsystems.intake.IntakeConstants.Spindexer;

public interface SpindexerIO extends SubsystemIO {

    /**
     * Spins the spindexer at the specified speed
     *
     * @param speed The speed to spin the spindexer at in radians per second
     */
    void go(AngularVelocity speed);

    /**
     * Stops the spindexer motor
     */
    void stop();

    /**
     * Gets the angular velocity of the spindexer
     *
     * @return The angular velocity of the spindexer as {@link AngularVelocity}
     */
    AngularVelocity getVelocity();

    public class Real implements SpindexerIO {
        LazyTalon motor;

        /**
         * Constructs a real instance of the Spindexer IO
         */
        public Real() {
            motor = new LazyTalonBuilder(
                Spindexer.MOTOR_ID, Spindexer.CAN_BUS_NAME,
                Spindexer.SENSOR_TO_MECHANISM_RATIO, Spindexer.INVERTED,
                Spindexer.STATOR_CURRENT_LIMIT.in(Amps), Spindexer.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Spindexer.P, Spindexer.I, Spindexer.D,
                    Spindexer.S, Spindexer.G, Spindexer.V, Spindexer.A,
                    Spindexer.FF_GRAVITY_TYPE, Spindexer.FF_SIGN,
                    0
                )
                .withMotionMagicConfiguration(
                    0, 0,
                    Spindexer.CRUISE_VELOCITY,
                    Spindexer.ACCELERATION
                )
                .build();

            // Superstructure.orchestra.addInstrument(this.motor.getMotor());
        }

        /**
         * Spins the spindexer at the specified speed
         *
         * @param velocity The speed to spin the spindexer as an {@link AngularVelocity}
         */
        public void go(AngularVelocity velocity) {
            motor.setPIDVelocityTarget(velocity, 0);
        }

        /**
         * Stops the spindexer motor
         */
        public void stop() {
            motor.stop();
        }

        /**
         * Gets the angular velocity of the spindexer
         *
         * @return The angular velocity of the spindexer as {@link AngularVelocity}
         */
        public AngularVelocity getVelocity() {
            return motor.getVelocity();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(Spindexer.TELEMETRY);
        }

    }

    public class Sim extends Real {}
}