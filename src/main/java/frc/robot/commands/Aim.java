package frc.robot.commands;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.command2.Command;
import frc.robot.subsystems.turret.AimBot;
import frc.robot.subsystems.turret.Turret;

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
