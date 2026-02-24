package raidzero.robot.subsystems.intake;

import edu.wpi.first.units.measure.AngularVelocity;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;

import static edu.wpi.first.units.Units.Amps;
import static raidzero.robot.subsystems.intake.IntakeConstants.Intake;

public interface IntakeIO extends SubsystemIO {

    void go(AngularVelocity speed);

    void stop();

    AngularVelocity getVelocity();

    public class Real implements IntakeIO {
        LazyTalon motor;

        public Real() {
            motor = new LazyTalonBuilder(
                    Intake.MOTOR_ID, Intake.CAN_BUS_NAME,
                    Intake.SENSOR_TO_MECHANISM_RATIO, Intake.INVERTED,
                    Intake.STATOR_CURRENT_LIMIT.in(Amps), Intake.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withPIDFConfiguration(
                            Intake.P, Intake.I, Intake.D,
                            Intake.S, Intake.G, Intake.V, Intake.A,
                            Intake.FF_GRAVITY_TYPE, Intake.FF_SIGN,
                            0)
                    .withMotionMagicConfiguration(
                            0, 0,
                            Intake.CRUISE_VELOCITY,
                            Intake.ACCELERATION)
                    .withFollower(Intake.FOLLOW_ID, Intake.CAN_BUS_NAME, Intake.FOLLOW_ALIGNMENT)
                    .build();

            // Superstructure.orchestra.addInstrument(this.motor.getMotor());
        }

        public void go(AngularVelocity velocity) {
            motor.setMMVelocityTarget(velocity, 0);
        }

        public void stop() {
            motor.stop();
        }

        public AngularVelocity getVelocity() {
            return motor.getVelocity();
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(Intake.TELEMETRY);
        }

    }

    public class Sim extends Real {
    }
}
