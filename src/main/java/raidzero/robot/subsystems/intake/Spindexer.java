package raidzero.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Spindexer extends R0Subsystem<SpindexerIO> {
    private static Spindexer system;

    /**
     * Runs the spindexer at the set index speed
     * 
     * @return a {@link Command}
     */
    public Command go() {
        return run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED));
    }

    /**
     * Stops the spindexer
     * 
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Constructs a Spindexer instance with the given IO implementation
     * 
     * @param io the IO implementation to use
     */
    private Spindexer(SpindexerIO io) {
        super(io);
    }

    /**
     * Gets the singleton instance of the Spindexer subsystem, creating it if it doesn't exist
     * @return the singleton instance of the Spindexer subsystem
     */
    public static Spindexer system() {
        if (system == null) {
            if (com.ctre.phoenix6.Utils.isSimulation())
                system = new Spindexer(new SpindexerIO.Sim());
            else
                system = new Spindexer(new SpindexerIO.Real());
        }
        return system;
    }
}
