// FIXME: Migrate this from a class-based command to a coroutine-based command.

package frc.robot.commands;

import static org.wpilib.units.Units.Rotations;

import java.util.Set;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import frc.robot.subsystems.turret.AimBot;
import frc.robot.subsystems.turret.Turret;

public class Aim implements Command {
    private final Turret turret = Turret.system();

    /**
     * Constructs the {@link Aim} command
     */
    public Aim() {}

    @Override
    public String name() {
        return "Aim";
    }

    @Override
    public Set<Mechanism> requirements() {
        return Set.of(turret);
    }

    @Override
    public void run(Coroutine coroutine) {
        while (true) {
            AimBot aquireAim = turret.getTurretAim(true);
            double robotRelativeTurretSetpoint = aquireAim.goal().getRotation().minus(aquireAim.robot().getRotation())
                .getRotations();

            turret.moveTo(Rotations.of(robotRelativeTurretSetpoint));
            coroutine.yield();
        }
    }

    @Override
    public void onCancel() {
        turret.stop();
    }
}
