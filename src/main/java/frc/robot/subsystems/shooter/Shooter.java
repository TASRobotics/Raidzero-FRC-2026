package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.Trigger;
import frc.lib.R0Subsystem;
import frc.robot.subsystems.shooter.ShooterConstants.Flywheel;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants.Kinetics;

public class Shooter extends R0Subsystem<ShooterIO> {
    private static Shooter system;
    private AngularVelocity velocityTarget = RotationsPerSecond.of(35);

    /**
     * Runs the shooter at the necessary speed from the RPS interpolating table
     *
     * @return A {@link Command} that runs the shooter at the target speed
     */
    public Command shoot() {
        return run(() -> {
            AngularVelocity target = RotationsPerSecond
                .of(Kinetics.RPS_TABLE.get(Turret.system().getHubDistance(true).in(Meters)));

            io.setVelocity(target);
        });
    }

    /**
     * Returns a {@link Trigger} that activates when the shooter is at it's target
     * velocity from the RPS interpolating table
     *
     * @return a {@link Trigger}
     */
    public Trigger atTargetVelocity() {
        return new Trigger(() -> {
            AngularVelocity target = RotationsPerSecond
                .of(Kinetics.RPS_TABLE.get(Turret.system().getHubDistance(true).in(Meters)));

            return io.getVelocity().minus(target).abs(RotationsPerSecond) < Flywheel.VELOCIOTY_TOLERANCE
                .in(RotationsPerSecond);
        });
    }

    /**
     * Returns a {@link Trigger} that activates when the shooter's velocity is less
     * than the velocity tolerance
     * stopped
     *
     * @return A {@link Trigger}
     */
    public Trigger stopped() {
        return new Trigger(
            () -> io.getVelocity().abs(RotationsPerSecond) < Flywheel.VELOCIOTY_TOLERANCE.in(RotationsPerSecond)
        );
    }

    /**
     * Returns a {@link Trigger} that activates when the shooter is at its target
     * velocity
     *
     * @see {@link #incrementVelocitySetpoint(AngularVelocity)}
     *
     * @return A {@link Trigger} that activates when the shooter is at its target
     *         velocity
     */
    public Trigger atManualTargetVelocity() {
        return new Trigger(() -> {
            return io.getVelocity().minus(this.velocityTarget).abs(RotationsPerSecond) < Flywheel.VELOCIOTY_TOLERANCE
                .in(RotationsPerSecond);
        });
    }

    /**
     * Runs the shooter at the velocity at its target velocity
     *
     * @see {@link #incrementVelocitySetpoint(AngularVelocity)}
     *
     * @return A {@link Command} that runs the shooter at the velocity at its target
     *         velocity
     */
    public Command manualShoot() {
        return run(() -> io.setVelocity(velocityTarget));
    }

    /**
     * Increments the velocity setpoint by the specified amount
     *
     * @param speed The amount to increment the velocity setpoint by
     * @return A {@link Command} that increments the velocity setpoint by the
     *         specified amount
     */
    public Command incrementVelocitySetpoint(AngularVelocity speed) {
        return runOnce(() -> velocityTarget = velocityTarget.plus(speed));
    }

    /**
     * Runs the shooter at the specified speed
     *
     * @param speed The speed at which to run the shooter
     * @return A {@link Command} that runs the shooter at the specified speed
     */
    public Command shoot(AngularVelocity speed) {
        this.velocityTarget = speed;
        return run(() -> io.setVelocity(speed));
    }

    /**
     * Sets the shooter speed to the specified percentage
     *
     * @param percent The shooter speed in percent
     * @return A {@link Command} that sets the shooter speed to the specified
     *         percentage
     */
    public Command setSpeed(double percent) {
        return run(() -> io.setPercent(percent));
    }

    /**
     * Stops the shooter motor
     *
     * @return A {@link Command} that stops the shooter motor
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
        SmartDashboard.putNumber(
            "Shooter speed",
            io.getVelocity().in(RotationsPerSecond)
        );
        SmartDashboard.putNumber(
            "Shooter target speed",
            velocityTarget.in(RotationsPerSecond)
        );
    }
}