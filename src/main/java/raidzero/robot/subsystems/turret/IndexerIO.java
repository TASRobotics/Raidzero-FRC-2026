package raidzero.robot.subsystems.turret;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;

import static edu.wpi.first.units.Units.Amps;
import static raidzero.robot.subsystems.turret.TurretConstants.Indexer.*;

public interface IndexerIO extends SubsystemIO {

    void go(double percent);
    void stop();

    public class Real implements IndexerIO {
        LazyTalon motor;

        public Real() {
            motor = new LazyTalon(MOTOR_ID, SENSOR_TO_MECHANISM_RATIO, INVERTED, STATOR_CURRENT_LIMIT.in(Amps),
                    SUPPLY_CURRENT_LIMIT.in(Amps)).build();
        }

        public void go(double percent) {
            motor.setPercent(percent);
        }

        public void stop() {
            motor.stop();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(null);
        }

    }

    public class Sim extends Real {
    }
}
