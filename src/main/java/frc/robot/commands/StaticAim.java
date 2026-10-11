package frc.robot.commands;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.turret.Turret;

public class StaticAim extends Command {
    private final Turret turret = Turret.system();
    Angle target;

    /**
     * Creates A {@link Command} that aims the turret to a static angle
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
