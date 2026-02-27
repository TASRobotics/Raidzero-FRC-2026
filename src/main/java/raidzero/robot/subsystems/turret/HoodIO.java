package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static raidzero.robot.subsystems.turret.TurretConstants.Pitch;

import edu.wpi.first.units.measure.Angle;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyFXS;
import raidzero.lib.wrappers.motors.LazyFXSBuilder;
import raidzero.robot.Superstructure;

public interface HoodIO extends SubsystemIO {
    /**
     * Moves the yaw to the specified pitch
     *
     * @param angleTarget the {@Link Angle}
     */
    public void moveTo(Angle angleTarget);

    /**
     * Gets the current ptich angle
     *
     * @return the {@link Angle}
     */
    public Angle getPitch();

    /**
     * Sets the startup angle for the hood
     */
    public void setStartupAngle();

    /**
     * Stops the hood motor
     */
    public void stop();

    public class Real implements HoodIO {
        LazyFXS pitch;

        public Real() {
            pitch = new LazyFXSBuilder(
                Pitch.MOTOR_ID, Pitch.CAN_BUS_NAME,
                Pitch.MOTOR_ARRANGEMENT,
                Pitch.SENSOR_TO_MECHANISM_RATIO,
                Pitch.INVERTED,
                Pitch.STATOR_CURRENT_LIMIT.in(Amps),
                Pitch.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Pitch.P, Pitch.I, Pitch.D,
                    Pitch.S, Pitch.G, Pitch.V, Pitch.A,
                    Pitch.FF_GRAVITY_TYPE,
                    Pitch.FF_SIGN,
                    Pitch.POSITION_SLOT
                )
                .withMotionMagicConfiguration(
                    Pitch.EXPO_V, Pitch.EXPO_A,
                    Pitch.CRUISE_VELOCITY,
                    Pitch.ACCELERATION
                )
                .build();

            // Superstructure.orchestra.addInstrument(pitch.getMotor());
        }

        @Override
        public void moveTo(Angle angleTarget) {
            pitch.setMMExpoTarget(angleTarget, 0);
        }

        @Override
        public Angle getPitch() {
            return pitch.getPosition();
        }

        @Override
        public void setStartupAngle() {
            pitch.getMotor().setPosition(Pitch.STARTING_ANGLE);
        }

        @Override
        public void updateTelemetry() {
            this.pitch.updateTelemetry(Pitch.TELEMETRY);
        }

        @Override
        public void stop() {
            this.pitch.stop();
        }
    }

    public class Sim extends Real {}

}