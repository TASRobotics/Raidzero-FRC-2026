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
    private Angle manualAngleTarget = Degrees.of(10);

    /**
     * Aims the hood to the pitch from the interpolation table based on the virtual goal distance.
     *
     * @return a {@link Command} that continuously updates the hood angle
     */
    public Command aim() {
        return run(() -> {
            double virtualDistance = Turret.system().getHubDistance(true).in(Meters);
            Angle target = Degrees.of(Kinetics.PITCH_TABLE.get(virtualDistance));

            SmartDashboard.putNumber("Hood/Target Pitch Deg", target.in(Degrees));

            io.moveTo(target);
        });
    }

    /**
     * Aims the hood to the specified fixed angle.
     *
     * @param angle the target {@link Angle}
     * @return a {@link Command} that holds the hood at the specified angle
     */
    public Command aim(Angle angle) {
        return run(() -> io.moveTo(angle));
    }

    /**
     * Aims the hood to the manually incremented angle target.
     * Use {@link #incrementPitch} to adjust the target during debug sessions.
     *
     * @return a {@link Command} that holds the hood at the manual target
     */
    public Command manualAim() {
        return run(() -> io.moveTo(manualAngleTarget));
    }

    /**
     * Increments the manual angle target by the given amount.
     *
     * @param angle the {@link Angle} to add (can be negative)
     * @return a {@link Command}
     */
    public Command incrementPitch(Angle angle) {
        return runOnce(() -> manualAngleTarget = manualAngleTarget.plus(angle));
    }

    /**
     * Stops the hood motor.
     *
     * @return a {@link Command}
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Seeds the hood encoder from the absolute sensor. Call once at startup.
     */
    public void setStartupAngle() {
        io.setStartupAngle();
    }

    /**
     * Returns the current hood pitch angle.
     *
     * @return current pitch as an {@link Angle}
     */
    public Angle getAngle() {
        return io.getPitch();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Pitch", io.getPitch().in(Degrees));

        SmartDashboard.putNumber("LUT Target Pitch", angleTarget.in(Degrees));
    }

    private Hood(HoodIO io) {
        super(io);
    }

    /**
     * Returns the singleton {@link Hood} instance, creating it if necessary.
     *
     * @return the {@link Hood} instance
     */
    public static Hood system() {
        if (system == null) {
            system = Utils.isSimulation()
                ? new Hood(new HoodIO.Sim())
                : new Hood(new HoodIO.Real());
        }
        return system;
    }
}