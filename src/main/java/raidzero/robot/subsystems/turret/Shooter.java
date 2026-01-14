package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.turret.TurretConstants.Setpoints;

public class Shooter extends R0Subsystem<ShooterIO> {
    private static Shooter system;
    private static Turret turret = Turret.system();

    /**
     * Runs the shooter at the necessary speed from the RPS interpolating table
     * @return a {@link Command}
     */
    public Command shoot() {
        return run(() -> {
            Distance hubDistance = turret.getHubDistance();
            AngularVelocity speed = RotationsPerSecond.of(Setpoints.RPS_TABLE.get(hubDistance.in(Meters)));

            io.setVelocity(speed);
        });
    }

    /**
     * Sets the shooter velocity to the specified angular velocity
     * @param rps the shooter velocity 
     * @return a {@link Command}
     */
    public Command setVelocity(AngularVelocity rps) {
        return run(() -> io.setVelocity(rps));
    }

    /**
     * Sets the shooter speed to the specified percentage
     * @param percent the shooter speed in percent
     * @return a {@link Command}
     */
    public Command setSpeed(double percent) {
        return run(() -> io.setPercent(percent));
    }

    /**
     * Stops the shooter motor
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    private Shooter(ShooterIO io) {
        super(io);
    }

    public static Shooter system() {
        if (system == null) {
            if (Utils.isSimulation()) {
                system = new Shooter(new ShooterIO.Sim());
            } else {
                system = new Shooter(new ShooterIO.Real());
            }
        }

        return system;
    }
}
