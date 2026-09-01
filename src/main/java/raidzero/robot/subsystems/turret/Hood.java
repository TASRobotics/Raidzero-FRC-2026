package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;

public class Hood extends R0Subsystem<HoodIO> {
    private static Hood system;
    private Angle angleTarget = Degrees.of(10);

    /**
     * Aims to the angle target
     *
     * @return A {@link Command} that aims to the angle target
     */
    public Command manualAim() {
        return run(() -> io.moveTo(angleTarget));
    }

    /**
     * Gets the current hood angle
     *
     * @return The current hood angle as an {@link Angle}
     */
    public Angle getAngle() {
        return io.getPitch();
    }

    /**
     * Aims to the specified angle
     *
     * @param angle the {@link Angle}
     * @return A {@link Command} that aims to the specified angle
     */
    public Command aim(Angle angle) {
        return run(() -> io.moveTo(angle));
    }

    /**
    * Aims to the angle from the RPS interpolating table based on the distance to the hub
    *
    * @return A {@link Command} that aims to the angle from the RPS interpolating table based on the distance to the hub
    */
    public Command aim() {
        return run(() -> {
            // Angle target = Degrees.of(Kinetics.PITCH_TABLE.get(Turret.system().getHubDistance(true).in(Meters)));

            // SmartDashboard.putNumber("Target pitch", target.in(Degrees));
            io.moveTo(angleTarget);
            // io.moveTo(target);
        });
    }

    /**
     * Stops the hood motor
     *
     * @return A {@link Command} that stops the hood motor
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Sets the startup angle
     */
    public void setStartupAngle() {
        io.setStartupAngle();
    }

    /**
     * Increments the target setpoint for manual aiming
     *
     * @param angle the {@link Angle} increment
     * @return A {@link Command} that increments the target setpoint for manual
     *         aiming
     */
    public Command incrementPitch(Angle angle) {
        return runOnce(() -> angleTarget = angleTarget.plus(angle));
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Pitch", io.getPitch().in(Degrees));

        SmartDashboard.putNumber("LUT Target Pitch", angleTarget.in(Degrees));
    }

    /**
     * Constructs a new {@link Hood} subsystem
     *
     * @param io The {@link HoodIO} implementation to use for this subsystem
     */
    private Hood(HoodIO io) {
        super(io);
    }

    /**
     * Gets the {@link Hood} subsystem instance
     *
     * @return The {@link Hood} subsystem instance
     */
    public static Hood system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Hood(new HoodIO.Real());
            else
                system = new Hood(new HoodIO.Sim());
        }

        return system;
    }

}