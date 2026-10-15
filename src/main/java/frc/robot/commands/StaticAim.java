package frc.robot.commands;

import static org.wpilib.units.Units.Rotations;

import java.util.Set;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;

import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.turret.Turret;

public class StaticAim implements Command {
    private final Turret turret = Turret.system();
    private Angle target;

    /**
     * Creates A {@link Command} that aims the turret to a static angle
     * @param target The target angle for the turret as an {@link Angle}
     */
    public StaticAim(Angle target) {
        this.target = target;
    }

    @Override
    public String name() {
        return "Static aim";
    }

    @Override
    public Set<Mechanism> requirements() {
        return Set.of(turret);
    }

    @Override
    public void run(Coroutine coroutine) {
        while (true) {
            Rotation2d swerveRotation = Swerve.system().getState().Pose.getRotation();
            Rotation2d targetRotation = new Rotation2d(target);
            double robotRelativeSetpoint = targetRotation.minus(swerveRotation).getRotations();

            turret.moveTo(Rotations.of(robotRelativeSetpoint));
            coroutine.yield();
        }
    }

    @Override
    public void onCancel() {
        turret.stop();
    }
}
