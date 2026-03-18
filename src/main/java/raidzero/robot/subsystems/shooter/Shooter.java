package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.shooter.ShooterConstants.Flywheel;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;

public class Shooter extends R0Subsystem<ShooterIO> {
    private static Shooter system;

    private AngularVelocity velocityTarget = RotationsPerSecond.of(35);
    private AngularVelocity autoVelocityTarget = RotationsPerSecond.of(0);
    private AngularVelocity activeTarget = RotationsPerSecond.of(0);

    /**
     * Runs the shooter at the RPS required for the current virtual goal distance.
     * The virtual goal distance comes from the kinetic
     * {@link raidzero.robot.subsystems.turret.AimBot},
     * which already accounts for robot velocity, turret mount offset, and time of
     * flight via the
     * virtual goal iteration in
     * {@link raidzero.robot.subsystems.turret.AimBot#adjustKineticLead}.
     *
     * @return a {@link Command} that continuously updates the flywheel setpoint
     */
    public Command shoot() {
        return run(() -> {
            double virtualDistance = Turret.system().getHubDistance(true).in(Meters);

            this.autoVelocityTarget = RotationsPerSecond.of(Kinetics.RPS_TABLE.get(virtualDistance));
            this.activeTarget = this.autoVelocityTarget;

            io.setVelocity(this.activeTarget);
        });
    }

    /**
     * Returns a {@link Trigger} that activates when the flywheel is within
     * tolerance of the
     * active target velocity (auto mode).
     *
     * @return a {@link Trigger}
     */
    public Trigger atTargetVelocity() {
        return new Trigger(() -> io.getVelocity().minus(this.activeTarget)
                .abs(RotationsPerSecond) < Flywheel.VELOCITY_TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Returns a {@link Trigger} that activates when the flywheel velocity is below
     * the
     * tolerance threshold (effectively stopped).
     *
     * @return a {@link Trigger}
     */
    public Trigger stopped() {
        return new Trigger(
                () -> io.getVelocity().abs(RotationsPerSecond) < Flywheel.VELOCITY_TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Returns a {@link Trigger} that activates when the flywheel is within
     * tolerance of the manual velocity target (set via
     * {@link #incrementVelocitySetpoint}).
     *
     * @return a {@link Trigger}
     */
    public Trigger atManualTargetVelocity() {
        return new Trigger(() -> io.getVelocity().minus(this.velocityTarget)
                .abs(RotationsPerSecond) < Flywheel.VELOCITY_TOLERANCE.in(RotationsPerSecond));
    }

    /**
     * Runs the shooter at the manually tuned velocity setpoint.
     *
     * @see #incrementVelocitySetpoint(AngularVelocity)
     * @return a {@link Command}
     */
    public Command manualShoot() {
        return run(() -> {
            this.activeTarget = velocityTarget;
            io.setVelocity(activeTarget);
        });
    }

    /**
     * Increments the manual velocity setpoint by the given amount. Intended for
     * tuning on the fly during debug sessions.
     *
     * @param speed the amount to add (can be negative)
     * @return a {@link Command}
     */
    public Command incrementVelocitySetpoint(AngularVelocity speed) {
        return runOnce(() -> velocityTarget = velocityTarget.plus(speed));
    }

    /**
     * Runs the shooter at a specific fixed speed, overriding both the manual and
     * auto targets.
     *
     * @param speed the target angular velocity
     * @return a {@link Command}
     */
    public Command shoot(AngularVelocity speed) {
        return run(() -> {
            this.velocityTarget = speed;
            this.activeTarget = speed;
            io.setVelocity(speed);
        });
    }

    /**
     * Sets the shooter to the given duty-cycle percentage. Active target is zeroed
     * so
     * {@link #atTargetVelocity()} does not false-trigger.
     *
     * @param percent duty cycle [0, 1]
     * @return a {@link Command}
     */
    public Command setSpeed(double percent) {
        return run(() -> {
            this.activeTarget = RotationsPerSecond.of(0);
            io.setPercent(percent);
        });
    }

    /**
     * Stops the flywheel and zeros the active target.
     *
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> {
            this.activeTarget = RotationsPerSecond.of(0);
            io.stop();
        });
    }

    private Shooter(ShooterIO io) {
        super(io);
    }

    /**
     * Returns the singleton {@link Shooter} instance, creating it if necessary.
     *
     * @return the {@link Shooter} instance
     */
    public static Shooter system() {
        if (system == null) {
            system = Utils.isSimulation()
                    ? new Shooter(new ShooterIO.Sim())
                    : new Shooter(new ShooterIO.Real());
        }
        return system;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Shooter/Speed RPS", io.getVelocity().in(RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/Target RPS", activeTarget.in(RotationsPerSecond));
        SmartDashboard.putNumber("Shooter/Auto Target RPS", autoVelocityTarget.in(RotationsPerSecond));
    }
}