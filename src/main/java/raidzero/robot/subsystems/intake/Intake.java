package raidzero.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Intake extends R0Subsystem<IntakeIO> {
    private static Intake system;

    /**
     * Runs the intake motors at the configured speed
     *
     * @return A {@link Command} that runs the intake
     */
    public Command go() {
        return run(() -> io.go(IntakeConstants.Intake.INTAKE_SPEED));
    }

    /**
     * Stops the intake
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Runs the intake at the anti-jam speed for a short duration to attempt to clear jams
     *
     * @return A {@link Command} that runs the intake at the anti-jam speed
     */
    public Command antiJam() {
        return run(() -> io.go(IntakeConstants.Intake.INTAKE_SPEED))
            .withTimeout(0.1);
    }

    /**
     * Constructs the {@link Intake} subsystem
     *
     * @param io The {@link IntakeIO} implementation to use for this subsystem
     */
    private Intake(IntakeIO io) {
        super(io);
    }

    /**
     * Gets the {@link Intake} subsystem instance
     *
     * @return The {@link Intake} subsystem instance
     */
    public static Intake system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Intake(new IntakeIO.Sim());
            else
                system = new Intake(new IntakeIO.Real());
        }
        return system;
    }

}
