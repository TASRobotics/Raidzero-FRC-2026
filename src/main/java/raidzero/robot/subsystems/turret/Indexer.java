package raidzero.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Indexer extends R0Subsystem<IndexerIO> {
    private static Indexer system;


    public Command go(double percent) {
        return run(() -> io.go(percent));
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    private Indexer(IndexerIO io) {
        super(io);
    }

    public static Indexer system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Indexer(new IndexerIO.Sim());
            else
                system = new Indexer(new IndexerIO.Real());
        }
        return system;
    }
}
