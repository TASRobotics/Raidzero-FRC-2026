package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Spindexer extends R0Subsystem<SpindexerIO> {
    private static Spindexer system;

    /**
     * Spins the spindexer at the index speed
     *
     * @return A {@link Command} that spins the spindexer at the index speed
     */
    public Command go() {
        return run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED));
    }

    public Command reverse() {
        return run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED.times(-1.0)));
    }

    /**
     * Runs the spindexer at the anti-jam speed for a short duration to attempt to
     * clear jams
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a
     *         short duration
     */
    public Command antiJam() {
        return run(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
            .withTimeout(0.1);
    }

    /**
     * Runs the spindexer at the anti-jam speed for a longer duration to attempt to
     * clear more severe jams
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a
     *         longer duration
     */
    public Command antiJamBig() {
        return run(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
            .withTimeout(0.2)
            .andThen(
                run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED)).withTimeout(0.3)
            )
            .repeatedly();
    }

    /**
     * AntiJam for Auto
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a
     *         longer duration
     */
    public Command antiJamAuto() {
        return run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED))
            .withTimeout(1)
            .andThen(
                run(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
                    .withTimeout(1)
            )
            .andThen(
                run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED))
                    .withTimeout(1)
            )
            .repeatedly();
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
     * Gets the angular velocity of the spindexer
     *
     * @return The angular velocity of the spindexer as an {@link AngularVelocity}
     */
    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    /**
     * Creates a {@link Trigger} that is active when the spindexer is at the target
     * velocity (index speed)
     *
     * @return a {@link Trigger}
     */
    public Trigger atTargetVelocity() {
        return new Trigger(
            () -> io.getVelocity().minus(IntakeConstants.Spindexer.INDEX_SPEED)
                .abs(RotationsPerSecond) < IntakeConstants.Spindexer.VELOCITY_TOLERANCE.in(RotationsPerSecond)
        );
    }

    /**
     * Creates a {@link Trigger} that is active when the spindexer is stopped (velocity
     *
     * @return
     */
    public Trigger stopped() {
        return new Trigger(
            () -> io.getVelocity().abs(RotationsPerSecond) < IntakeConstants.Spindexer.VELOCITY_TOLERANCE
                .in(RotationsPerSecond)
        );
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
