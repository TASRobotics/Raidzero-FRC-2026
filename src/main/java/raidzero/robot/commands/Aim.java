package raidzero.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.turret.AimBot;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants.Yaw;

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

        double currentRotations = turret.getCurrentYaw().in(Rotations);

        double bestSetpoint = findBestWrap(
            currentRotations,
            robotRelativeTurretSetpoint
        );

        turret.moveTo(Rotations.of(bestSetpoint));
    }

    private double findBestWrap(double currentRotations, double desiredRotations) {
        double revLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);
        double fwdLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);
        double bestSetpoint = currentRotations;
        double bestDistance = Double.MAX_VALUE;

        for (int i = -2; i <= 2; i++) {
            double candidate = desiredRotations + i;
            if (candidate >= revLimit && candidate <= fwdLimit) {
                double dist = Math.abs(candidate - currentRotations);
                if (dist < bestDistance) {
                    bestDistance = dist;
                    bestSetpoint = candidate;
                }
            }
        }
        return bestSetpoint;
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
