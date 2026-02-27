package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;
import raidzero.robot.subsystems.shooter.ShooterConstants.Feeder;

public interface FeederIO extends SubsystemIO {

    void go(AngularVelocity speed);

    void stop();

    AngularVelocity getVelocity();

    public class Real implements FeederIO {
        LazyTalon motor;

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

        public void go(AngularVelocity velocity) {
            motor.setPIDVelocityTarget(velocity, 0);
        }

        public void stop() {
            motor.stop();
        }

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
