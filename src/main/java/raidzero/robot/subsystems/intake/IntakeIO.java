package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static raidzero.robot.subsystems.intake.IntakeConstants.Intake;

import edu.wpi.first.units.measure.AngularVelocity;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;

public interface IntakeIO extends SubsystemIO {
    /**
     * Runs the intake motors at the given speed
     *
     * @param speed The speed to run the intake motors at as an {@link AngularVelocity}
     */
    void go(AngularVelocity speed);

    /**
     * Stops the intake motors
     */
    void stop();

    /**
     * Gets the angular velocity of the intake motors
     *
     * @return The angular velocity of the intake motors as an {@link AngularVelocity}
     */
    AngularVelocity getVelocity();

    public class Real implements IntakeIO {

        /**
         * Constructs the real instance of the Intake IO (Hardware disabled)
         */
        public Real() {
            // Hardware disabled
        }

        @Override
        public void go(AngularVelocity velocity) {}

        @Override
        public void stop() {}

        @Override
        public AngularVelocity getVelocity() {
            return edu.wpi.first.units.Units.RotationsPerSecond.of(0);
        }

        @Override
        public void updateTelemetry() {}
    }

    public class Sim extends Real {}
}
