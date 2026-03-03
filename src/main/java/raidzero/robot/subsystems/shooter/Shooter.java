package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;

public class Shooter extends R0Subsystem<ShooterIO> {
    private static Shooter system;
    private AngularVelocity velocityTarget = RotationsPerSecond.of(35);

    /**
     * Runs the shooter at the necessary speed from the RPS interpolating table
     *
     * @return a {@link Command}
     */
    public Command shoot() {
        return run(() -> {
            AngularVelocity target = RotationsPerSecond
                .of(Kinetics.RPS_TABLE.get(Turret.system().getHubDistance(true).in(Meters)));

            io.setVelocity(target);
        });
    }

    /**
     * Runs the shooter at the velocity set by the {@link #velocityTarget} variable,
     * which can be changed with the
     * {@link #incrementVelocitySetpoint(AngularVelocity)} method
     *
     * @return a {@link Command}
     */
    public Command manualShoot() {
        return run(() -> io.setVelocity(velocityTarget));
    }

    /**
     * Increments the velocity setpoint by the specified amount, which will affect
     * the speed of the shooter when the {@link #manualShoot()} command is running
     *
     * @param speed the amount to increment the velocity setpoint by
     * @return a {@link Command}
     */
    public Command incrementVelocitySetpoint(AngularVelocity speed) {
        return runOnce(() -> velocityTarget = velocityTarget.plus(speed));
    }

    /**
     * Runs the shooter at the specified speed
     * @param speed the speed at which to run the shooter
     * @return a {@link Command}
     */
    public Command shoot(AngularVelocity speed) {
        this.velocityTarget = speed;
        return run(() -> io.setVelocity(speed));
    }

    /**
     * Sets the shooter speed to the specified percentage
     *
     * @param percent the shooter speed in percent
     * @return a {@link Command}
     */
    public Command setSpeed(double percent) {
        return run(() -> io.setPercent(percent));
    }

    /**
     * Stops the shooter motor
     *
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Constructs the {@link Shooter} subsystem
     *
     * @param io The {@link ShooterIO} implementation to use for this subsystem
     */
    private Shooter(ShooterIO io) {
        super(io);
    }

    /**
     * Gets the {@link Shooter} subsystem instance
     *
     * @return The {@link Shooter} subsystem instance
     */
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

    @Override
    public void periodic() {
        super.periodic();
        // SmartDashboard.putNumber("Shooter speed", io.getVelocity().in(RotationsPerSecond));
        // SmartDashboard.putNumber("Shooter target speed", velocityTarget.in(RotationsPerSecond));
    }

}