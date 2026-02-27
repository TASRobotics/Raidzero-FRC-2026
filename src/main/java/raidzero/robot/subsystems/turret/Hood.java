package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.lib.R0Subsystem;

public class Hood extends R0Subsystem<HoodIO> {
    private static Hood system;
    private Angle angleTarget = Degrees.of(10);

    /**
     * Aims to the angle target
     *
     * @return a {@link Command}
     */
    public Command manualAim() {
        return run(() -> io.moveTo(angleTarget));
    }

    /**
     * Aims to the specified angle
     *
     * @param angle the {@link Angle}
     * @return a {@link Command}
     */
    public Command aim(Angle angle) {
        return run(() -> io.moveTo(angle));
    }

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
     * @return
     */
    public Command incrementPitch(Angle angle) {
        return runOnce(() -> angleTarget = angleTarget.plus(angle));
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Target pitch", angleTarget.in(Degrees));
        SmartDashboard.putNumber("Pitch", io.getPitch().in(Degrees));
    }

    private Hood(HoodIO io) {
        super(io);
    }

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