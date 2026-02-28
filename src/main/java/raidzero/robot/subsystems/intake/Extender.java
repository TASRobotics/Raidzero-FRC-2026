package raidzero.robot.subsystems.intake;

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
     * Stops the intake extension
     * 
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * A {@link Trigger} that is active when the intake is fully extended
     * 
     * @return a {@link Trigger}
     */
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
     * Constructs an Extender instance with the given IO implementation
     * 
     * @param io the IO implementation to use
     */
    private Extender(ExtenderIO io) {
        super(io);
    }

    /**
     * Returns the Extender singleton instance, creating it if it does not exist.
     * Chooses the appropriate IO implementation based on if the robot is in sim or
     * not
     * 
     * @return the Extender singleton instance
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
