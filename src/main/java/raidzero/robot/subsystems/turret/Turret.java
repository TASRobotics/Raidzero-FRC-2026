package raidzero.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;

import raidzero.lib.R0Subsystem;

public class Turret extends R0Subsystem<TurretIO> {
    private static Turret system;

    private Turret(TurretIO io) {
        super(io);
    }

    public Turret getSystem() {
        if (system == null) {
            if (Utils.isSimulation()) system = new Turret(new TurretIO.Sim());
            else system = new Turret(new TurretIO.Real());
        }
        return system;
    } 
}
