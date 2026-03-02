package raidzero.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.turret.AimBot;
import raidzero.robot.subsystems.turret.Turret;

public class Aim extends Command {
    private final Turret turret = Turret.system();

    /**
     * Constructs the {@link Aim} command
     */
    public Aim() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        AimBot aquireAim = turret.getTurretAim(true);
        double robotRelativeTurretSetpoint = aquireAim.goal().getRotation().minus(aquireAim.robot().getRotation())
            .getRotations();

        turret.moveTo(Rotations.of(robotRelativeTurretSetpoint));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
