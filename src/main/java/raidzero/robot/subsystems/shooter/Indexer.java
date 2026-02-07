package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Indexer extends R0Subsystem<IndexerIO> {
    private static Indexer system;


    public Command go(AngularVelocity speed) {
        return run(() -> io.go(speed));
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
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Indexer Velocity", io.getVelocity().in(RotationsPerSecond));
    }
}
