package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static raidzero.robot.subsystems.intake.IntakeConstants.Extender;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;

public interface ExtenderIO extends SubsystemIO {
    /**
     * Extends the intake outwards
     */
    void extend();

    /**
     * Retracts the intake inwards
     */
    void retract();

    /**
     * Stops the intake extension motor
     */
    void stop();

    /**
     * Sets the starting angle for the intake
     */
    void setStartupAngle();

    /**
     * Checks if the intake is fully extended
     * 
     * @return true if the intake is fully extended, false otherwise
     */
    boolean isExtended();

    public class Real implements ExtenderIO {
        LazyTalon motor;

        public Real() {
            motor = new LazyTalonBuilder(
                Extender.MOTOR_ID, Extender.CAN_BUS_NAME,
                Extender.SENSOR_TO_MECHANISM_RATIO, Extender.INVERTED,
                Extender.STATOR_CURRENT_LIMIT.in(Amps), Extender.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Extender.P, Extender.I, Extender.D,
                    Extender.S, Extender.G, Extender.V, Extender.A,
                    Extender.FF_GRAVITY_TYPE, Extender.FF_SIGN,
                    0
                )
                .withMotionMagicConfiguration(
                    Extender.EXPO_V, Extender.EXPO_A,
                    Extender.CRUISE_VELOCITY,
                    Extender.ACCELERATION
                )
                .withFollower(Extender.FOLLOW_ID, Extender.CAN_BUS_NAME, Extender.FOLLOW_ALIGNMENT)
                .withSoftLimits(
                    Extender.ENALBE_FORWARD_SOFT_LIMIT, Extender.FORWARD_SOFT_LIMIT.in(Rotations),
                    Extender.ENABLE_REVERSE_SOFT_LIMIT, Extender.REVERSE_SOFT_LIMIT.in(Rotations)
                )
                .build();

            // Superstructure.orchestra.addInstrument(this.motor.getMotor());
        }

        @Override
        public void extend() {
            motor.setMMExpoTarget(Extender.EXTENDED_SETPOINT, 0);
        }

        @Override
        public void retract() {
            motor.setMMExpoTarget(Extender.RETRACTED_SETPOINT, 0);
        }

        @Override
        public void stop() {
            motor.stop();
        }

        @Override
        public void setStartupAngle() {
            motor.getMotor().setPosition(Extender.STARTUP_ANGLE);
        }

        @Override
        public boolean isExtended() {
            double distance = Math.abs(motor.getPosition().in(Rotations) - Extender.EXTENDED_SETPOINT.in(Rotations));
            return distance < Extender.ANGLE_TOLERANCE.in(Rotations);
        }

        @Override
        public void updateTelemetry() {
            motor.updateTelemetry(Extender.TELEMETRY);
        }
    }

    public class Sim extends Real {}
}
