package raidzero.robot.subsystems.turret;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static raidzero.robot.subsystems.turret.TurretConstants.Indexer.*;
import static raidzero.robot.subsystems.turret.TurretConstants.Shooter.TELEMETRY;

import com.ctre.phoenix6.signals.GravityTypeValue;


public interface IndexerIO extends SubsystemIO {

    void go(double percent);
    void stop();

    public class Real implements IndexerIO {
        LazyTalon motor;

        public Real() {
            motor = new LazyTalon(MOTOR_ID, SENSOR_TO_MECHANISM_RATIO, INVERTED, STATOR_CURRENT_LIMIT.in(Amps),
                    SUPPLY_CURRENT_LIMIT.in(Amps)).build();
                    motor.withMotionMagicConfiguration(P, I, D, S, G, V, A, 0, 0, GravityTypeValue.Elevator_Static, CRUISE_VELOCITY.in(RotationsPerSecond), ACCELERATION.in(RotationsPerSecondPerSecond));
        }

        public void go(double percent) {
            motor.setPercent(percent);
        }

        public void stop() {
            motor.stop();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(TELEMETRY);
        }

    }

    public class Sim extends Real {
    }
}
