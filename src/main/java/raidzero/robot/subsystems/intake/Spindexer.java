package raidzero.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Spindexer extends R0Subsystem<SpindexerIO> {
    private static Spindexer system;

    public Command go() {
        return run(() -> io.go(IntakeConstants.Spindexer.INDEX_SPEED));
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    public Spindexer(SpindexerIO io) {
        super(io);
    }

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
