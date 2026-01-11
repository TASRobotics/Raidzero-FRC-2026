package raidzero.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public void setPitch(double targetPitch) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPitch'");
    }
    
    public void setYaw(Rotation2d targetYaw) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setYaw'");
    }

    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    } 
}
