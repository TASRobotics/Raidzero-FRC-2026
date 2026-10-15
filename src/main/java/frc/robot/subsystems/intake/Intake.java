package frc.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import frc.lib.R0Subsystem;

public class Intake extends R0Subsystem<IntakeIO> {
    private static Intake system;

    /**
     * Runs the intake motors at the configured speed
     *
     * @return A {@link Command} that runs the intake
     */
    public Command go() {
        return runRepeatedly(() -> io.go(IntakeConstants.Intake.INTAKE_SPEED))
            .named("Intake go");
    }

    /**
     * Reverses the intake motors at the configured speed
     *
     * @return A {@link Command} that reverses the intake
     */
    public Command reverse() {
        return runRepeatedly(() -> io.go(IntakeConstants.Intake.INTAKE_SPEED.times(-1.0)))
            .named("Intake reverse");
    }

    /**
     * Stops the intake
     *
     * @return A {@link Command} that stops the intake
     */
    public Command stop() {
        return runRepeatedly(() -> io.stop())
            .named("Intake stop");
    }

    /**
     * Runs the intake at the anti-jam speed for a short duration to attempt to
     * clear jams
     *
     * @return A {@link Command} that runs the intake at the anti-jam speed
     */
    public Command antiJam() {
        return runRepeatedly(() -> io.go(IntakeConstants.Intake.INTAKE_SPEED))
            .named("Intake antijam")
            .withTimeout(Seconds.of(0.1));
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
