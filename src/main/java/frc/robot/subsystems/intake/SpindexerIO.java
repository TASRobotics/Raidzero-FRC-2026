package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;

import org.wpilib.units.measure.AngularVelocity;
import frc.lib.SubsystemIO;
import frc.lib.wrappers.motors.LazyTalon;
import frc.lib.wrappers.motors.LazyTalonBuilder;
import frc.robot.Superstructure;
import frc.robot.subsystems.intake.IntakeConstants.Spindexer;

public interface SpindexerIO extends SubsystemIO {

    /**
     * Runs the indexer at the specified speed
     *
     * @param speed the speed at which to run the indexer
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
                Spindexer.MOTOR_ID, Superstructure.canDoAttitude,
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
            motor.updateTelemetry(Spindexer.TELEMETRY);
        }

    }

    public class Sim extends Real {}
}