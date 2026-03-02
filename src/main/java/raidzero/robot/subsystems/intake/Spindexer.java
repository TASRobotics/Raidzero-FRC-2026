package raidzero.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
     * Runs the spindexer at the anti-jam speed for a short duration to attempt to clear jams
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a short duration
     */
    public Command antiJam() {
        return run(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
            .withTimeout(0.1);
    }

    /**
     * Runs the spindexer at the anti-jam speed for a longer duration to attempt to clear more severe jams
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a longer duration
     */
    public Command antiJamBig() {
        return run(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
            .withTimeout(0.1)
            .andThen(run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED)).withTimeout(0.3));
    }

    /**
     * Stops the spindexer
     *
     * @return A {@link Command} that stops the spindexer
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
