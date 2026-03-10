package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
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
     * Runs the shooter at the necessary speed from the RPS interpolating table
     *
     * @return A {@link Command} that runs the shooter at the target speed
     */
    public Command shoot() {
        return run(() -> {
            Turret turret = Turret.system();

            double distance = turret.getHubDistance(true).in(Meters);

            // Base RPS from table (stationary physics)
            double baseRPS = Kinetics.RPS_TABLE.get(distance);

            // Radial Velocity (accounts for robot movement)
            LinearVelocity vRadial = turret.getTurretAim(true)
                .getRadialVelocity(turret.getFilteredSpeeds());

            // Compensation: Subtract vRadial because if vRadial is moving away (negative)
            // we need to ADD RPS to the shooter
            double compensationRPS = -vRadial.in(MetersPerSecond) * Kinetics.VELOCITY_COMPENSATION_SCALAR;

            this.autoVelocityTarget = RotationsPerSecond.of(baseRPS + compensationRPS);
            this.activeTarget = this.autoVelocityTarget;

            io.setVelocity(this.activeTarget);
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
            return io.getVelocity().minus(this.activeTarget).abs(RotationsPerSecond) < Flywheel.VELOCIOTY_TOLERANCE
                .in(RotationsPerSecond);
        });
    }

    /**
     * Returns a {@link Trigger} that activates when the shooter is at its target velocity
     *
     * @see {@link #incrementVelocitySetpoint(AngularVelocity)}
     *
     * @return A {@link Trigger} that activates when the shooter is at its target velocity
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
     * @return A {@link Command} that runs the shooter at the velocity at its target velocity
     */
    public Command manualShoot() {
        return run(() -> {
            this.activeTarget = velocityTarget;
            io.setVelocity(activeTarget);
        });
    }

    /**
     * Increments the velocity setpoint by the specified amount
     *
     * @param speed The amount to increment the velocity setpoint by
     * @return A {@link Command} that increments the velocity setpoint by the specified amount
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
        return run(() -> {
            this.velocityTarget = speed;
            this.activeTarget = speed;
            io.setVelocity(speed);
        });
    }

    /**
     * Sets the shooter speed to the specified percentage
     *
     * @param percent The shooter speed in percent
     * @return A {@link Command} that sets the shooter speed to the specified percentage
     */
    public Command setSpeed(double percent) {
        return run(() -> {
            this.activeTarget = RotationsPerSecond.of(0);
            io.setPercent(percent);
        });
    }

    /**
     * Stops the shooter motor
     *
     * @return A {@link Command} that stops the shooter motor
     */
    public Command stop() {
        return run(() -> {
            this.activeTarget = RotationsPerSecond.of(0);
            io.stop();
        });
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
            activeTarget.in(RotationsPerSecond)
        );
        io.updateTelemetry();
    }
}