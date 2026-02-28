package raidzero.robot.subsystems.shooter;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import raidzero.lib.R0Subsystem;

public class Feeder extends R0Subsystem<FeederIO> {
    private static Feeder system;

    /**
     * Runs the feeder at the set feeder speed
     * 
     * @return a {@link Command}
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
     * Constructs a Feeder instance with the given IO implementation
     * 
     * @param io the IO implementation to use
     */
    private Feeder(FeederIO io) {
        super(io);
    }

    /**
     * Gets the singleton instance of the Feeder subsystem, creating it if it
     * doesn't exist
     * 
     * @return the singleton instance of the Feeder subsystem
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
