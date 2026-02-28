package raidzero.robot.subsystems.shooter;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RotationsPerSecond;

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
     * Stops the feeder motor
     * 
     * @return a {@link Command}
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
        SmartDashboard.putNumber("Indexer Velocity", io.getVelocity().in(RotationsPerSecond));
    }
}
