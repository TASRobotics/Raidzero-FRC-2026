package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
        LazyTalon motor;

        /**
         * Constructs the real instance of the Intake IO
         */
        public Real() {
            motor = new LazyTalonBuilder(
                Intake.MOTOR_ID, Superstructure.canDoAttitude,
                Intake.SENSOR_TO_MECHANISM_RATIO, Intake.INVERTED,
                Intake.STATOR_CURRENT_LIMIT.in(Amps), Intake.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Intake.P, Intake.I, Intake.D,
                    Intake.S, Intake.G, Intake.V, Intake.A,
                    Intake.FF_GRAVITY_TYPE, Intake.FF_SIGN,
                    0
                )
                .withPIDFConfiguration(
                    Intake.P_1, Intake.I_1, Intake.D_1,
                    Intake.S_1, Intake.G_1, Intake.V_1, Intake.A_1,
                    Intake.FF_GRAVITY_TYPE, Intake.FF_SIGN,
                    1
                )
                .withMotionMagicConfiguration(
                    0, 0,
                    Intake.CRUISE_VELOCITY,
                    Intake.ACCELERATION
                )
                .withFollower(Intake.FOLLOW_ID, Superstructure.canDoAttitude, Intake.FOLLOW_ALIGNMENT)
                .build();
            motor.setCoast();

            // Superstructure.orchestra.addInstrument(this.motor.getMotor());
        }

        @Override
        public void go(AngularVelocity velocity) {
            motor.setVelocityTorqueCurrentTarget(velocity, Intake.A_1, 1);
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
            motor.updateTelemetry(Intake.TELEMETRY);
        }

    }

    public class Sim extends Real {}

    /**
     * Creates no motors and ignores every request, for running without the intake
     */
    public class Disabled implements IntakeIO {
        @Override
        public void go(AngularVelocity velocity) {}

        @Override
        public void stop() {}

        @Override
        public AngularVelocity getVelocity() {
            return RotationsPerSecond.of(0);
        }

        @Override
        public void updateTelemetry() {}
    }
}
