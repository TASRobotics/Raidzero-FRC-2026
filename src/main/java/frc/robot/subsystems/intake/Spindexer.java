package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import frc.lib.R0Subsystem;

public class Spindexer extends R0Subsystem<SpindexerIO> {
    private static Spindexer system;

    /**
     * Spins the spindexer at the index speed
     *
     * @return A {@link Command} that spins the spindexer at the index speed
     */
    public Command go() {
        return runRepeatedly(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED))
            .named("Spndexer go");
    }

    /**
     * Spins the spindexer in reverse at the index speed
     *
     * @return A {@link Command} that spins the spindexer in reverse at the index speed
     */
    public Command reverse() {
        return runRepeatedly(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED.times(-1.0)))
            .named("Spindexer reverse");
    }

    /**
     * Runs the spindexer at the anti-jam speed for a short duration to attempt to
     * clear jams
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a
     *         short duration
     */
    public Command antiJam() {
        return run(coroutine -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
            .named("Spindexer antijam")
            .withTimeout(Seconds.of(0.1));
    }

    /**
     * Runs the spindexer at the anti-jam speed for a longer duration to attempt to
     * clear more severe jams
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a
     *         longer duration
     */
    public Command antiJamBig() {
        return run(coroutine -> {
            while (true) {
                coroutine.await(
                    runRepeatedly(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
                        .named("Spindexer antijam")
                        .withTimeout(Seconds.of(0.2))
                );
                coroutine.await(
                    runRepeatedly(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED))
                        .named("Spindexer intake")
                        .withTimeout(Seconds.of(0.3))
                );
                coroutine.yield();
            }
        })
            .named("Spindexer antijam big");
    }

    /**
     * AntiJam for Auto
     *
     * @return A {@link Command} that runs the spindexer at the anti-jam speed for a
     *         longer duration
     */
    public Command antiJamAuto() {
        return run(coroutine -> {
            while (true) {
                coroutine.await(
                    runRepeatedly(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED))
                        .named("Spindexer intake")
                        .withTimeout(Seconds.of(1))
                );

                coroutine.await(
                    runRepeatedly(() -> io.go(IntakeConstants.Spindexer.ANTIJAM_SPEED))
                        .named("Spindexer antijam")
                        .withTimeout(Seconds.of(1))
                );

                coroutine.await(
                    runRepeatedly(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED))
                        .named("Spindexer intake")
                        .withTimeout(Seconds.of(1))
                );

                coroutine.yield();
            }
        })
            .named("Spindexer auton antijam");
    }

    /**
     * Stops the spindexer
     *
     * @return A {@link Command} that stops the spindexer
     */
    public Command stop() {
        return runRepeatedly(() -> io.stop())
            .named("Spindexer stop");
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
