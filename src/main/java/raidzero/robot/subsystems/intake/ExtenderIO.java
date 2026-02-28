package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static raidzero.robot.subsystems.intake.IntakeConstants.Extender;

import edu.wpi.first.units.measure.Angle;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;

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
     * Agitates the intake at level 1
     */
    void agitateL1();

    /**
     * Agitates the intake at level 2
     */
    void agitateL2();

    /**
     * Agitates the intake at level 3
     */
    void agitateL3();

    /**
     * Stops the intake extension motor
     */
    void stop();

    /**
     * Sets the starting angle for the intake
     */
    void setStartupAngle();

    /**
     * Increases the agitation of the extender
     */
    void increaseAgitate();

    /**
     * Decreases the agitation of the extender
     */
    void decreaseAgitate();

    /**
     * Sets the encoder position for the extender
     *
     * @param position The new encoder position
     */
    void overrideEncoderPosition(Angle position);

    /**
     * Returns true if the extender is extended, false otherwise
     *
     * @return True if the extender is extended, false otherwise
     */
    boolean isExtended();

    public class Real implements ExtenderIO {
        LazyTalon motor;

        /**
         * Constructs a real instance of the Extender IO
         */
        public Real() {
            motor = new LazyTalonBuilder(
                Extender.MOTOR_ID, Superstructure.canDoAttitude,
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
                .withFollower(Extender.FOLLOW_ID, Superstructure.canDoAttitude, Extender.FOLLOW_ALIGNMENT)
                .withSoftLimits(
                    Extender.ENALBE_FORWARD_SOFT_LIMIT, Extender.FORWARD_SOFT_LIMIT.in(Rotations),
                    Extender.ENABLE_REVERSE_SOFT_LIMIT, Extender.REVERSE_SOFT_LIMIT.in(Rotations)
                )
                .build();
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
        public void agitateL1() {
            motor.setMMExpoTarget(Extender.AGITATE_ONE_SETPOINT, 0);
        }

        @Override
        public void agitateL2() {
            motor.setMMExpoTarget(Extender.AGITATE_TWO_SETPOINT, 0);
        }

        @Override
        public void agitateL3() {
            motor.setMMExpoTarget(Extender.AGITATE_THREE_SETPOINT, 0);
        }

        @Override
        public void stop() {
            motor.stop();
        }

        @Override
        public void increaseAgitate() {
            Extender.AGITATE_ONE_SETPOINT.plus(Rotations.of(0.01));
            Extender.AGITATE_TWO_SETPOINT.plus(Rotations.of(0.01));
            Extender.AGITATE_THREE_SETPOINT.plus(Rotations.of(0.01));
        }

        @Override
        public void decreaseAgitate() {
            Extender.AGITATE_ONE_SETPOINT.minus(Rotations.of(0.01));
            Extender.AGITATE_TWO_SETPOINT.minus(Rotations.of(0.01));
            Extender.AGITATE_THREE_SETPOINT.minus(Rotations.of(0.01));
        }

        @Override
        public void setStartupAngle() {
            motor.getMotor().setPosition(Extender.STARTUP_ANGLE);
        }

        @Override
        public void overrideEncoderPosition(Angle position) {
            motor.getMotor().setPosition(position);
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
