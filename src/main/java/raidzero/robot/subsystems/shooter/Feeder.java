package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Feeder extends R0Subsystem<FeederIO> {
    private static Feeder system;

    public Command go() {
        return run(() -> io.go(ShooterConstants.Feeder.FEEDER_SPEED));
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    private Feeder(FeederIO io) {
        super(io);
    }

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
