package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Feeder extends R0Subsystem<FeederIO> {
    private static Feeder system;

    /**
     * Runs the feeder at the configured speed
     *
     * @return A {@link Command} that runs the feeder
     */
    public Command go() {
        return run(() -> io.go(ShooterConstants.Feeder.FEEDER_SPEED));
    }

    /**
     * Runs the feeder in reverse at the configured speed
     *
     * @return A {@link Command} that reverses the feeder
     */
    public Command reverse() {
        return run(() -> io.go(ShooterConstants.Feeder.FEEDER_SPEED.times(-1.0)));
    }

    /**
     * Returns a {@link Trigger} that activates when the feeder is at its target
     * velocity from the configured speed
     *
     * @return a {@link Trigger}
     */
    public Trigger atTargetVelocity() {
        return new Trigger(() -> {
            return io.getVelocity().minus(ShooterConstants.Feeder.FEEDER_SPEED)
                .abs(RotationsPerSecond) < ShooterConstants.Feeder.VELOCITY_TOLERANCE.in(RotationsPerSecond);
        });
    }

    /**
     * Stops the feeder motor
     *
     * @return A {@link Command} that stops the feeder
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Constructs the {@link Feeder} subsystem
     *
     * @param io The {@link FeederIO} implementation to use for this subsystem
     */
    private Feeder(FeederIO io) {
        super(io);
    }

    /**
     * Gets the {@link Feeder} subsystem instance
     *
     * @return The {@link Feeder} subsystem instance
     */
    public static Feeder system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Feeder(new FeederIO.Sim());
            else
                system = new Feeder(new FeederIO.Real());
        }
        return system;
    }

    @Override
    public void periodic() {
        super.periodic();
        // SmartDashboard.putNumber("Indexer Velocity",
        // io.getVelocity().in(RotationsPerSecond));
    }

}
