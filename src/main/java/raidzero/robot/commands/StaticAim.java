package raidzero.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.Turret;

public class StaticAim extends Command {
    private final Turret turret = Turret.system();
    Angle target;

    /**
     * Creates a {@link Command} that aims the turret to a static angle
     * @param target The target angle for the turret as an {@link Angle}
     */
    public StaticAim(Angle target) {
        this.target = target;
        this.addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d swerveRotation = Swerve.system().getState().Pose.getRotation();
        Rotation2d targetRotation = new Rotation2d(target);
        double robotRelativeSetpoint = targetRotation.minus(swerveRotation).getRotations();

        turret.moveTo(Rotations.of(robotRelativeSetpoint));
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
