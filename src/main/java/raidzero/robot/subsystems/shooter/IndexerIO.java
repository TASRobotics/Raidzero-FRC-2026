package raidzero.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Amps;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.subsystems.shooter.ShooterConstants.Indexer;

public interface IndexerIO extends SubsystemIO {

    void go(AngularVelocity speed);

    void stop();

    AngularVelocity getVelocity();

    public class Real implements IndexerIO {
        LazyTalon motor;

        public Real() {
            motor = new LazyTalonBuilder(
                    Indexer.MOTOR_ID, Indexer.CAN_BUS_NAME,
                    Indexer.SENSOR_TO_MECHANISM_RATIO, Indexer.INVERTED,
                    Indexer.STATOR_CURRENT_LIMIT.in(Amps), Indexer.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withPIDFConfiguration(
                            Indexer.P, Indexer.I, Indexer.D,
                            Indexer.S, Indexer.G, Indexer.V, Indexer.A,
                            GravityTypeValue.Elevator_Static, StaticFeedforwardSignValue.UseClosedLoopSign,
                            0)
                    .withMotionMagicConfiguration(
                            Indexer.EXPO_V, Indexer.EXPO_A,
                            Indexer.CRUISE_VELOCITY,
                            Indexer.ACCELERATION)
                    .build();
        }

        public void go(AngularVelocity velocity) {
            motor.setVelocity(velocity, 0);
        }

        public void stop() {
            motor.stop();
        }

        public AngularVelocity getVelocity() {
            return motor.getFeedbackVelocity();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(Indexer.TELEMETRY);
        }

    }

    public class Sim extends Real {
    }
}
