package raidzero.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Intake extends R0Subsystem<IntakeIO> {
    private static Intake system;

    /**
     * Runs the intake at the specified speed
     * @return a {@link Command}
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
     * Constructs an Intake instance with the given IO implementation
     * 
     * @param io the IO implementation to use
     */
    private Intake(IntakeIO io) {
        super(io);
    }

    /**
     * Gets the singleton instance of the Intake subsystem, creating it if it doesn't exist
     * @return the singleton instance of the Intake subsystem
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
