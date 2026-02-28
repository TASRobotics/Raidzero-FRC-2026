package raidzero.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Spindexer extends R0Subsystem<SpindexerIO> {
    private static Spindexer system;

    /**
     * Spins the spindexer at the index speed
     *
     * @return a {@link Command} that spins the spindexer at the index speed
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
     * Constructs a new {@link Spindexer} subsystem
     *
     * @param io The {@link SpindexerIO} implementation to use for this subsystem
     */
    public Spindexer(SpindexerIO io) {
        super(io);
    }

    /**
     * Gets the {@link Spindexer} subsystem instance
     *
     * @return The {@link Spindexer} subsystem instance
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
