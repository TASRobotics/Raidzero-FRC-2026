package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Extender extends R0Subsystem<ExtenderIO> {
    private static Extender system;

    /**
     * Extends the intake outwards
     *
     * @return a {@link Command}
     */
    public Command extend() {
        return run(() -> io.extend());
    }

    /**
     * Retracts the intake inwards
     *
     * @return a {@link Command}
     */
    public Command retract() {
        return run(() -> io.retract());
    }

    /**
     * Stops the extender motors
     *
     * @return A {@link Command} that stops the extender
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    public Trigger isExtended() {
        return new Trigger(() -> io.isExtended());
    }

    /**
     * Sets the starting angle for the intake
     */
    public void setStartupAngle() {
        io.setStartupAngle();
    }

    /**
     * Zeros the encoder position for the extender
     *
     * @return A {@link Command} that zeros the encoder position
     */
    public Command zeroEncoderPosition() {
        return runOnce(() -> io.overrideEncoderPosition(Rotations.of(0.0)));
    }

    /**
     * Constructs the {@link Extender} subsystem
     *
     * @param io The {@link ExtenderIO} implementation to use for this subsystem
     */
    private Extender(ExtenderIO io) {
        super(io);
    }

    /**
     * Gets the {@link Extender} subsystem instance
     *
     * @return The {@link Extender} subsystem instance
     */
    public static Extender system() {
        if (system == null) {
            if (com.ctre.phoenix6.Utils.isSimulation())
                system = new Extender(new ExtenderIO.Sim());
            else
                system = new Extender(new ExtenderIO.Real());
        }
        return system;
    }
}
