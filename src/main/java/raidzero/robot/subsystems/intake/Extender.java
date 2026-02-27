package raidzero.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Extender extends R0Subsystem<ExtenderIO> {
    private static Extender system;

    /**
     * Extends the intake
     *
     * @return A {@link Command} that extends the intake
     */
    public Command extend() {
        return run(() -> io.extend());
    }

    /**
     * Retracts the intake
     *
     * @return A {@link Command} that retracts the intake
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

    /**
     * Checks if the extender is fully extended
     *
     * @return A {@link Trigger} that is active when the extender is fully extended
     */
    public Trigger isExtended() {
        return new Trigger(() -> io.isExtended());
    }

    /**
     * Sets the startup angle of the extender to the current position
     */
    public void setStartupAngle() {
        io.setStartupAngle();
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
