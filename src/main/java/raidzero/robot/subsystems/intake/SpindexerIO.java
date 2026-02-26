package raidzero.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Amps;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;

import static raidzero.robot.subsystems.intake.IntakeConstants.Spindexer;

public interface SpindexerIO extends SubsystemIO {

    void go(AngularVelocity speed);

    void stop();

    AngularVelocity getVelocity();

    public class Real implements SpindexerIO {
        LazyTalon motor;

        public Real() {
            motor = new LazyTalonBuilder(
                    Spindexer.MOTOR_ID, Spindexer.CAN_BUS_NAME,
                    Spindexer.SENSOR_TO_MECHANISM_RATIO, Spindexer.INVERTED,
                    Spindexer.STATOR_CURRENT_LIMIT.in(Amps), Spindexer.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withPIDFConfiguration(
                            Spindexer.P, Spindexer.I, Spindexer.D,
                            Spindexer.S, Spindexer.G, Spindexer.V, Spindexer.A,
                            Spindexer.FF_GRAVITY_TYPE, Spindexer.FF_SIGN,
                            0)
                    .withMotionMagicConfiguration(
                            0, 0,
                            Spindexer.CRUISE_VELOCITY,
                            Spindexer.ACCELERATION)
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
            motor.updateTelemetry(Spindexer.TELEMETRY);
        }

    }

    public class Sim extends Real {
    }
}