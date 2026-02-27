package raidzero.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Angle;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.subsystems.climb.ClimbConstants.Winch;

public interface ClimbIO extends SubsystemIO {
    /**
     * Moves the winch to the specified position
     *
     * @param position the {@link double} position in meters
     */
    void moveTo(Angle position);

    /**
     * Stops the climb motor
     */
    void stop();

    public class Real implements ClimbIO {
        LazyTalon motor;

        /**
         * Constructs a real instance of the Climb IO
         */
        public Real() {
            motor = new LazyTalonBuilder(
                Winch.MOTOR_ID, Winch.SENSOR_TO_MECHANISM_RATIO, Winch.INVERTED,
                Winch.STATOR_CURRENT_LIMIT.in(Amps), Winch.SUPPLY_CURRENT_LIMIT.in(Amps)
            ).build();
        }

        @Override
        public void moveTo(Angle position) {
            motor.setMMExpoTarget(position, 0);
        }

        @Override
        public void stop() {
            motor.stop();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(Winch.TELEMETRY);
        }

    }

    public class Sim extends Real {}
}
