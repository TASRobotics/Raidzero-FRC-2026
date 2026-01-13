package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.turret.TurretConstants.Setpoints;

public class Turret extends R0Subsystem<TurretIO> {
    private static Turret system;

    public Command aim() {
        return run(() -> {
            Angle yawOffset = io.getYawFromTarget();

            Angle currentYaw = io.getYaw();
            Angle targetYaw = currentYaw.plus(yawOffset);

            Distance distFromTarget = io.getHubDistance();
            Angle targetPitch = Rotations.of(Setpoints.PITCH_TABLE.get(distFromTarget.in(Meters)));

            io.moveTo(targetPitch, targetYaw);
        });

    }

    public Command moveTo(Angle pitch, Angle yaw) {
        return run(() -> io.moveTo(pitch, yaw));
    }

    public void setYaw(Angle yaw) {
        io.setYaw(yaw);
    }
    
    public void setPitch(Angle pitch) {
        io.setPitch(pitch);
    }

    public Command stop() {
        return run(() -> io.stopMotors());
    }

    public Distance getHubDistance() {
        return io.getHubDistance();
    }

    public Trigger hasHubLOS() {
        return new Trigger(() -> io.hasHubLOS());
    }

    private Turret(TurretIO io) {
        super(io);
    }

    public static Turret system() {
        if (system == null) {
            if (Utils.isSimulation()) {
                system = new Turret(new TurretIO.Sim());
            } else {
                system = new Turret(new TurretIO.Real());
            }
        }
        return system;
    }

}
