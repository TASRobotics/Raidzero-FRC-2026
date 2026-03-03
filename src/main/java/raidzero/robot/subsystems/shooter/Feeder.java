package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
     * Stops the feeder motor
     *
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Returns whether a ball is currently detected by the CANrange sensor
     *
     * @return true if a ball has been detected within the detection window
     */

    public Trigger ballDetected() {
        return new Trigger(() -> io.ballDetected());
    }

    public boolean isBallDetected() {
        return io.ballDetected();
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
        SmartDashboard.putBoolean("Ball detected", io.ballDetected());
    }
}
