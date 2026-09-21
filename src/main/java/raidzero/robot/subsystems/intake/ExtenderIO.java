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
        private Angle position = Extender.RETRACTED_SETPOINT;

        /**
         * Constructs a real instance of the Extender IO (Hardware disabled)
         */
        public Real() {
            // Hardware disabled
        }

        @Override
        public void extend() {
            position = Extender.EXTENDED_SETPOINT;
        }

        @Override
        public void retract() {
            position = Extender.RETRACTED_SETPOINT;
        }

        @Override
        public void agitateL1() {
            position = Extender.AGITATE_ONE_SETPOINT;
        }

        @Override
        public void agitateL2() {
            position = Extender.AGITATE_TWO_SETPOINT;
        }

        @Override
        public void agitateL3() {
            position = Extender.AGITATE_THREE_SETPOINT;
        }

        @Override
        public void stop() {}

        @Override
        public void increaseAgitate() {}

        @Override
        public void decreaseAgitate() {}

        @Override
        public void setStartupAngle() {
            position = Extender.STARTUP_ANGLE;
        }

        @Override
        public void overrideEncoderPosition(Angle position) {
            this.position = position;
        }

        @Override
        public boolean isExtended() {
            return position.equals(Extender.EXTENDED_SETPOINT);
        }

        @Override
        public void updateTelemetry() {}
    }

    public class Sim extends Real {}
}
